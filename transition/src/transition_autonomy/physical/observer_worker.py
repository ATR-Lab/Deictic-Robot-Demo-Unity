"""Process-isolated read-only I/O. Timeouts never imply sensor freshness.

Only a trusted observation-only factory is accepted by the composition root.
Never put motion RPCs here: terminating a process cannot retract a delivered RPC.
"""
from __future__ import annotations

import multiprocessing

from .manifest import finite


def _serve(connection, factory):
    observer = None
    try:
        observer = factory()
        while connection.recv() == 'read':
            try:
                # Plain immutable/serializable diagnostics; the parent validates
                # provenance before constructing a commissioned evidence bundle.
                connection.send(('sample', observer.read()))
            except Exception as error:
                connection.send(('error', type(error).__name__ + ': ' + str(error)))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        if observer is not None and callable(getattr(observer, 'close', None)):
            observer.close()
        connection.close()


class ReadOnlyWorker:
    def __init__(self, factory, *, timeout_s=.25):
        self.timeout_s = finite(timeout_s, 'observer timeout', positive=True)
        if self.timeout_s > 5:
            raise ValueError('Observer timeout exceeds bounded diagnostic maximum')
        context = multiprocessing.get_context('spawn')
        self._connection, child = context.Pipe()
        self._process = context.Process(target=_serve, args=(child, factory), daemon=True)
        self._process.start()
        child.close()
        self._fault = None

    def read(self):
        if self._fault:
            raise RuntimeError(self._fault)
        try:
            self._connection.send('read')
            if not self._connection.poll(self.timeout_s):
                raise TimeoutError('Observation worker deadline expired')
            kind, value = self._connection.recv()
            if kind != 'sample':
                raise RuntimeError(value)
            return value
        except (TimeoutError, OSError, EOFError, RuntimeError) as error:
            self._fault = str(error)
            self.close()
            raise RuntimeError(self._fault) from error

    def close(self):
        # This is an OBSERVATION worker; no move/stop is ever admitted here.
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(.2)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(.2)
        self._connection.close()
