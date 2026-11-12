"""Small owning CUDA/DLPack buffer using the installed CUDA driver."""
import ctypes as ct
import numpy as np


class CudaBuffer:
    def __init__(self, shape, dtype=np.float32, device=0):
        from ovphysx.dlpack import DLTensor, DLDevice, DLDataType
        self.shape = tuple(shape); self.dtype = np.dtype(dtype)
        self.bytes = int(np.prod(shape)) * self.dtype.itemsize
        self.lib = ct.CDLL('libcuda.so.1')
        self.context = ct.c_void_p(); self.pointer = ct.c_uint64()
        self._check(self.lib.cuInit(0))
        self._check(self.lib.cuDevicePrimaryCtxRetain(ct.byref(self.context), ct.c_int(device)))
        self._push()
        try:
            self._check(self.lib.cuMemAlloc_v2(ct.byref(self.pointer), ct.c_size_t(self.bytes)))
            self._check(self.lib.cuMemsetD8_v2(self.pointer, 0, ct.c_size_t(self.bytes)))
            self._check(self.lib.cuCtxSynchronize())
        finally:
            self._pop()
        self.device = device
        self._shape = (ct.c_int64 * len(shape))(*shape)
        self.tensor = DLTensor()
        self.tensor.data = self.pointer.value
        self.tensor.device = DLDevice(); self.tensor.device.device_type = 2
        self.tensor.device.device_id = device
        self.tensor.ndim = len(shape); self.tensor.shape = self._shape
        self.tensor.dtype = DLDataType()
        self.tensor.dtype.code = 2 if self.dtype.kind == 'f' else 0
        self.tensor.dtype.bits = self.dtype.itemsize * 8; self.tensor.dtype.lanes = 1

    def _check(self, code):
        if code:
            message = ct.c_char_p()
            self.lib.cuGetErrorString(ct.c_int(code), ct.byref(message))
            raise RuntimeError(f'CUDA driver error {code}: {message.value!r}')

    def _push(self):
        self._check(self.lib.cuCtxPushCurrent_v2(self.context))

    def _pop(self):
        old = ct.c_void_p()
        self._check(self.lib.cuCtxPopCurrent_v2(ct.byref(old)))

    def numpy(self):
        result = np.empty(self.shape, self.dtype)
        self._push()
        try:
            self._check(self.lib.cuCtxSynchronize())
            self._check(self.lib.cuMemcpyDtoH_v2(ct.c_void_p(result.ctypes.data),
                                                self.pointer, ct.c_size_t(self.bytes)))
        finally:
            self._pop()
        return result

    def close(self):
        if self.pointer.value:
            self._push()
            try: self._check(self.lib.cuMemFree_v2(self.pointer))
            finally: self._pop()
            self.pointer.value = 0
            self._check(self.lib.cuDevicePrimaryCtxRelease_v2(ct.c_int(self.device)))
