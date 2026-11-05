"""Failure categories shared by the benchmark. No third-party imports."""

# Frame/visit categories kept identical to the prototype, plus explicit stop reasons.
ARUCO_TARGET_NOT_DETECTED = 'ARUCO_TARGET_NOT_DETECTED'
ARUCO_GRIPPER_NOT_DETECTED = 'ARUCO_GRIPPER_NOT_DETECTED'
INVALID_POSE_ESTIMATE = 'INVALID_POSE_ESTIMATE'
CAMERA_CAPTURE_FAILURE = 'CAMERA_CAPTURE_FAILURE'
DUPLICATE_FRAME = 'DUPLICATE_FRAME'
MOTION_FAILURE = 'MOTION_FAILURE'
SAFETY_REJECTION = 'SAFETY_REJECTION'
PERSISTENCE_FAILURE = 'PERSISTENCE_FAILURE'
INTERRUPTED = 'INTERRUPTED'
OTHER = 'OTHER'

VISION_CATEGORIES = {ARUCO_TARGET_NOT_DETECTED, ARUCO_GRIPPER_NOT_DETECTED,
                     INVALID_POSE_ESTIMATE, CAMERA_CAPTURE_FAILURE, DUPLICATE_FRAME}


class BenchmarkError(RuntimeError):
    category = OTHER

    def __init__(self, message, category=None):
        super().__init__(message)
        if category is not None:
            self.category = category


class MotionFailure(BenchmarkError):
    category = MOTION_FAILURE


class SafetyRejection(BenchmarkError):
    category = SAFETY_REJECTION


class PersistenceFailure(BenchmarkError):
    category = PERSISTENCE_FAILURE


class CameraFailure(BenchmarkError):
    category = CAMERA_CAPTURE_FAILURE


class Interrupted(BenchmarkError):
    category = INTERRUPTED


def category_of(error):
    if isinstance(error, BenchmarkError):
        return error.category
    if isinstance(error, KeyboardInterrupt):
        return INTERRUPTED
    if isinstance(error, (OSError, MemoryError)):
        return PERSISTENCE_FAILURE
    return OTHER
