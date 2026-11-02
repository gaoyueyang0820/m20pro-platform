"""Optional explicitly built C ABI reducer. Import never compiles or downloads.

try_reduce returns None for absent/incompatible libraries or marginal numerical
inputs; the existing NumPy producer remains the authority for those frames.
"""
import ctypes
import os
from pathlib import Path
import threading
import numpy as np

ABI=1
EXTENT=32
SIDE=65
CELLS=SIDE*SIDE
_lock=threading.RLock()
_library=None
_load_key=None
_load_error=None
_stats={'calls':0,'native_frames':0,'fallback_frames':0,'last_reason':None,'fallback_reasons':{}}
_reasons={1:'unsupported_input',2:'nondefault_rounding_mode',3:'radius_boundary',4:'ground_height_boundary',
          5:'height_bin_boundary',6:'insufficient_ground',7:'sample_height_boundary',8:'narrow_ground_sample',
          9:'degenerate_ground',10:'slope_boundary_or_steep',11:'rounded_height_boundary',12:'classification_boundary',13:'allocation_failed'}


class _Result(ctypes.Structure):
    _fields_=[('abi',ctypes.c_uint32),('reason',ctypes.c_int32),('input_points',ctypes.c_uint64),
              ('floor_height',ctypes.c_double),('coeff',ctypes.c_double*3),('condition',ctypes.c_double)]


def _path():
    return Path(os.environ.get('M20_SENSING_LIBRARY') or Path(__file__).resolve().parent/'native'/'libsensing.so').expanduser()


def _load():
    global _library,_load_key,_load_error
    path=_path()
    try:stat=path.stat();key=(str(path),stat.st_mtime_ns,stat.st_size)
    except OSError:key=(str(path),None,None)
    with _lock:
        if key==_load_key:return _library
        _load_key=key;_library=None;_load_error=None
        try:
            lib=ctypes.CDLL(str(path))
            lib.m20_sensing_abi_version.argtypes=[];lib.m20_sensing_abi_version.restype=ctypes.c_uint32
            lib.m20_sensing_result_size.argtypes=[];lib.m20_sensing_result_size.restype=ctypes.c_size_t
            if lib.m20_sensing_abi_version()!=ABI or lib.m20_sensing_result_size()!=ctypes.sizeof(_Result):
                raise ValueError('native sensing ABI mismatch')
            lib.m20_sensing_reduce.argtypes=[ctypes.c_void_p,ctypes.c_uint64,ctypes.c_uint32,
                ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_uint32,ctypes.POINTER(_Result)]
            lib.m20_sensing_reduce.restype=ctypes.c_int
            _library=lib
        except (OSError,AttributeError,ValueError) as exc:_load_error=str(exc)
        return _library


def _fallback(reason):
    with _lock:
        _stats['fallback_frames']+=1;_stats['last_reason']=reason
        counts=_stats['fallback_reasons'];counts[reason]=counts.get(reason,0)+1
    return None


def try_reduce(points):
    """Reduce a complete Nx3 native-endian float32/64 array, or return None."""
    with _lock:_stats['calls']+=1
    lib=_load()
    if lib is None:return _fallback('library_unavailable')
    if not isinstance(points,np.ndarray) or points.ndim!=2 or points.shape[1]!=3 or len(points)<100:
        return _fallback('unsupported_input')
    if points.dtype not in [np.dtype('float32'),np.dtype('float64')]:return _fallback('unsupported_dtype')
    data=np.ascontiguousarray(points)
    floor=np.empty(CELLS,np.uint8);obstacle=np.empty(CELLS,np.uint8);bounds=np.empty((CELLS,4),np.float64);result=_Result()
    code=lib.m20_sensing_reduce(data.ctypes.data,len(data),data.dtype.itemsize,
        floor.ctypes.data,obstacle.ctypes.data,bounds.ctypes.data,CELLS,ctypes.byref(result))
    if code or result.abi!=ABI:return _fallback(_reasons.get(result.reason,'native_error'))
    measured=np.flatnonzero(floor);occupied=np.flatnonzero(obstacle);clear=np.flatnonzero(floor&~obstacle)
    unpack=lambda ids:np.column_stack([ids//SIDE-EXTENT,ids%SIDE-EXTENT]).astype(int).tolist()
    output=dict(floor=unpack(clear),obstacles=unpack(occupied),obstacle_bounds=np.round(bounds[occupied],4).tolist(),
                measured_floor=unpack(measured),geometry_version=2,resolution=.2,
                floor_height=round(float(result.floor_height),3),input_points=int(result.input_points))
    with _lock:_stats['native_frames']+=1;_stats['last_reason']=None
    return output


def backend_status():
    lib=_load()
    with _lock:
        return dict(available=lib is not None,abi=ABI,library=str(_path()),load_error=_load_error,
                    **{k:(dict(v) if isinstance(v,dict) else v) for k,v in _stats.items()})
