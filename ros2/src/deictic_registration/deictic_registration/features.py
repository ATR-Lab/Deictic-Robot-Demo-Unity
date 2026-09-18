"""SuperPoint extraction and LightGlue matching, with explicit dependencies.

Upstream: https://github.com/cvg/LightGlue
The paper specifies SuperPoint but does not name its descriptor matcher;
LightGlue is a documented implementation choice, not a claimed paper setting.
"""

import numpy as np


class SuperPointMatcher:
    def __init__(self, device="cpu", max_keypoints=512, cpu_threads=4, match_filter_threshold=0.1,
                 cuda_memory_fraction=0.25):
        if not np.isfinite(match_filter_threshold) or not 0 <= match_filter_threshold <= 1:
            raise ValueError("match_filter_threshold must be in [0, 1]")
        if not np.isfinite(cuda_memory_fraction) or not 0 < cuda_memory_fraction <= 1:
            raise ValueError("cuda_memory_fraction must be finite and in (0, 1]")
        try:
            import torch
            from lightglue import LightGlue, SuperPoint
        except ImportError as exc:
            raise RuntimeError(
                "SuperPoint provider unavailable: install the pinned LightGlue frontend "
                "and PyTorch in the ROS Python environment; no dummy fallback exists.") from exc
        self.torch = torch
        self.device = torch.device(device)
        if cpu_threads < 1:
            raise ValueError("cpu_threads must be positive")
        # Preprocessing and tensor transfers also use CPU work in CUDA mode.
        torch.set_num_threads(cpu_threads)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested for SuperPoint but unavailable")
        if self.device.type == "cuda":
            # Bound this process's PyTorch allocator before either model moves
            # to CUDA. Driver/context allocations are outside this allocator cap.
            device_index = self.device.index if self.device.index is not None else torch.cuda.current_device()
            torch.cuda.set_per_process_memory_fraction(cuda_memory_fraction, device=device_index)
        # Upstream downloads published pretrained weights into torch's model cache.
        self.extractor = SuperPoint(max_num_keypoints=max_keypoints).eval().to(self.device)
        self.matcher = LightGlue(features="superpoint", filter_threshold=match_filter_threshold).eval().to(self.device)

    def match(self, headset_rgb, wrist_rgb):
        from lightglue.utils import rbd

        def tensor(rgb):
            rgb = np.asarray(rgb)
            if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
                raise ValueError("Feature input must be an HxWx3 uint8 RGB image")
            return self.torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1))).to(
                device=self.device, dtype=self.torch.float32) / 255.0

        with self.torch.inference_mode():
            # Keep original pixel coordinates/resolution for aligned depth lookup.
            features0 = self.extractor.extract(tensor(headset_rgb), resize=None)
            features1 = self.extractor.extract(tensor(wrist_rgb), resize=None)
            matched = self.matcher({"image0": features0, "image1": features1})
            features0, features1, matched = [rbd(x) for x in (features0, features1, matched)]
            pairs = matched["matches"]
            return (features0["keypoints"][pairs[:, 0]].cpu().numpy(),
                    features1["keypoints"][pairs[:, 1]].cpu().numpy())
