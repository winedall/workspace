"""里程计 adapter 集合。

每个模型一个 adapter 文件，由注册表的 adapter_path 选择；
类名按文件名 PascalCase + "OdometryAdapter" 约定。
"""

from .base import BaseOdometryAdapter, OdometryContext
from .prev_joint_pos import PrevJointPosOdometryAdapter
from .z01_std_odom_no_acc import Z01StdOdomNoAccOdometryAdapter
from .z01_std_odom_with_acc import Z01StdOdomWithAccOdometryAdapter

__all__ = [
    "BaseOdometryAdapter",
    "OdometryContext",
    "PrevJointPosOdometryAdapter",
    "Z01StdOdomNoAccOdometryAdapter",
    "Z01StdOdomWithAccOdometryAdapter",
]
