# rm_connecting_rod_gripper_controller

## 物理原理

夹爪实物图

![](./assets/gripper_example.jpg)

夹爪通过电机直接驱动，然后联动齿轮旋转控制外面的夹爪片

规定让夹爪闭合为正方向

## parameters

参数通过 generate_parameter_library 生成

+ gripper_motor 类型string，夹爪电机的motor名称，没有默认值，必填
+ left_connecting_rod_joint 类型string，为左边连杆处的关节名称，没有默认值，必填
+ right_connecting_rod_joint 类型string，为右边连杆处的关节名称，没有默认值，必填
+ close_position 类型为 double，当gripper_motor的位置到这里时认为夹住
+ open_position 类型为 double, 当gripper_motor的位置到这里时认为打开
+ kp/kd/ki/max_output/max_integral/static_friction 类型都为 double，这些为位置控制闭环参数。
+ hold_effort 类型为 double，为夹爪关闭后持续施加的力，用作夹紧。这个力必须为正。
+ position_threshold 类型为double，当|夹爪位置-目标位置|<=  position_threshold 时认为到达
+ debug 类型为 bool，默认为 false,打开将会提供debug topic信息

## controller functions

控制器需要声明以下接口：

state interface:
+ gripper_motor/position
+ gripper_motor/velocity
+ gripper_motor/effort

command interface:
+ gripper_motor/effort

控制器需要暴露以下接口：

state interface:
+ left_connecting_rod_joint/position
+ left_connecting_rod_joint/velocity
+ right_connecting_rod_joint/position
+ right_connecting_rod_joint/velocity

计算方式如下：

left_connecting_rod_joint/position = right_connecting_rod_joint/position = gripper_motor/position
left_connecting_rod_joint/velocity = right_connecting_rod_joint/velocity = gripper_motor/velocity

控制器需要提供topic用于分别接受打开关闭的指令。所有指令以最新到达的为准，初始使用 open 作为当前指令。也就是初始夹爪默认开启。

夹爪的状态如下
+ opened
+ opening
+ closed
+ closing

opening/closing 的进入条件为通过topic控制，closed从closing转变，opened从opening转变，两个均遵顼position_threshold的转变规则

控制器需要持续发布gripper控制状态


控制器需要提供以下debug功能：

+ 发布全部的joint的ref/state信息。
+ 发布全部的pid信息

## application detail

+ 使用 generate_parameter_library 进行参数生成
+ 使用自己实现的pid，并且实现通过topic动态调参
+ 在每一次完成功能后，调用subagent进行代码审查 