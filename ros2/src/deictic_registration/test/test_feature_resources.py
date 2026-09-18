"""Resource policy checks without loading weights or allocating a GPU."""
import sys
from types import ModuleType, SimpleNamespace
import pytest
from deictic_registration.features import SuperPointMatcher


@pytest.fixture
def fake_dependencies(monkeypatch):
    calls=[]
    torch=ModuleType('torch')
    torch.device=lambda name:SimpleNamespace(type=name.split(':')[0],name=name,
        index=int(name.split(':')[1]) if ':' in name else None)
    torch.set_num_threads=lambda count:calls.append(('threads',count))
    def set_fraction(fraction,device):
        # PyTorch2.7 requires an explicit integer/index for this API; an
        # unindexed torch.device('cuda') raises instead of using current device.
        assert isinstance(device,int)
        calls.append(('cap',fraction,device))
    torch.cuda=SimpleNamespace(is_available=lambda:True,current_device=lambda:2,
        set_per_process_memory_fraction=set_fraction)
    lightglue=ModuleType('lightglue')
    class Model:
        def __init__(self, **kwargs):
            calls.append(('model',kwargs))
        def eval(self):
            return self
        def to(self,device):
            calls.append(('allocate',device.name))
            return self
    lightglue.SuperPoint=lightglue.LightGlue=Model
    monkeypatch.setitem(sys.modules,'torch',torch)
    monkeypatch.setitem(sys.modules,'lightglue',lightglue)
    return calls


@pytest.mark.parametrize('fraction',[0.,-0.1,1.01,float('nan'),float('inf')])
def test_invalid_allocator_fraction_rejects_before_model_or_device_work(fake_dependencies,fraction):
    with pytest.raises(ValueError,match='cuda_memory_fraction'):
        SuperPointMatcher('cuda',cuda_memory_fraction=fraction)
    assert fake_dependencies==[]


def test_cuda_cap_precedes_both_model_allocations_on_selected_device(fake_dependencies):
    SuperPointMatcher('cuda:1',cuda_memory_fraction=.2)
    assert ('threads',4) in fake_dependencies
    cap_index=fake_dependencies.index(('cap',.2,1))
    assert all(cap_index < i for i,c in enumerate(fake_dependencies) if c[0] in ('model','allocate'))
    assert [c for c in fake_dependencies if c[0]=='allocate']==[('allocate','cuda:1')]*2


def test_cpu_keeps_thread_setting_and_never_initializes_cuda_allocator(fake_dependencies):
    SuperPointMatcher('cpu',cpu_threads=3)
    assert fake_dependencies[0]==('threads',3)
    assert not any(c[0]=='cap' for c in fake_dependencies)


def test_default_cuda_fraction_is_quarter(fake_dependencies):
    SuperPointMatcher('cuda')
    assert ('cap',.25,2) in fake_dependencies
