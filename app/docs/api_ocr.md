

## Page 1

KUKA

Kuka-interface 标准接口
目录

Kuka-interface 标准接口 .4 1
-- AMR 任务执行相关接口 ecseeesseeesneesssecsneesssecsneesneessnecsusesnessneesneesasessueesneesnecsneesusecseesusesneessnseeneensnetens 3
1任务下发 Dispatch Mission (上游系统 -> Kuka 系统 ) ceececcsescscscscscscscscscsescscsescscsesesesesesesscsesesesesesesceeees 3
公共请求参数说明 .4 3
1.1 货架移动任务(missionType=RACK_MOVE) ....ccccccccsescsccscsscscscscscssscssescessessescessesusssessssuessesessuessesaeens 5
1.2 辊简车搬运任务(missionType=ROLLER_MOVE) ......cccssescssesscsssssescsscscsscssesesessecsesenssestsacssstsasssansneees 6
1.3 料箱搬运(missionType=PICKER_MOVE) 8
1.4 叉车搬运任务(missionType=FORKLIFT_MOVE) ....scscsccsessssessescscsscscsssscsessssscsscsssesscscsssestssessseseceaes 10
1.5 机器人移动(missionType=MOVE) u.....cssecescecceccecsceccscsescscscscseevevavavacacacacecscecscsceceseseseseststststseevens 11
1.6 复合机器人任务(missionType=ROBOTICS_MOVE) Ne 12
2 任务取消 Cancel Mission (上游系统 -> Kuka 系统 ) eececescesccsesseesecseeseeseesesseaseaseaeatesteateseeseesseseeseeses 14
3 任务放行 Resume Workflow (EV RA -> Kuka 系统 ) cecccccccsssscssesssseesesssecsesessecsescsessescseessseeseescaes 15
4作业看板查询接口 Query Jobs (上游系统 -> Kuka 系统 ) cece cess cscscsescsseseescssssseeeeesseseeaes 16
-- AMIR 容器相关接口 eesseecsneessseesneesssccsneesssecsneesusessnessusesnsessuessnsesnsessueesusessueesusesneesueesneessueesnteeseeesneenees 19
5 容器入场 Insert Container (上游系统 -> Kuka 系统 ) ceeeeccccscsssssssesescssssesceecsescsscsesceseseeneseeeeeaeeneaeeeees 19
6 容器出场 Remove Container (上游系统 -> Kuka 系统 ) ceececcsccssscssessscssesescessscssesescesesesesssceseseseeseseaee 21
7 容器信息更新 Update Container (暂时支持: 位置和空满状态的更新) 〈上游系统 -> Kuka 系
统 ) cecssssssesseessecsssesueesuessvecssecunesueesucesnecsesseesucesuessuesasecaesusesueesuessuesueesuessuetsesseesueesusesueeseeatesutesneesneesseesnetes 22
8 接口回调 Mission Status Callback (Kuka 系统 -> 上游系统 ) cceccccscscscsssscscsceceesescsescsceesesesesesteeeeneees 23
9 容器模型查询接口 Query Container Model Codes (上游系统 -> Kuka 系统) eee 24
10 查询容器模型推荐的可存放区域 Query Area Codes By Conatiner Model (上游系统 -> Kuka 系
统 ) cecssssssesseessecsssesueesuessvecssecunesueesucesnecsesseesucesuessuesasecaesusesueesuessuesueesuessuetsesseesueesusesueeseeatesutesneesneesseesnetes 24
11-1 容器信息查询接口(仅查询入场状态容器) Query Containers (上游系统 -> Kuka 系统) eee. 25
11-2 容器信息查询接口(同时查询入场和离场状态容器) Query Containers (上游系统 -> Kuka 系统)
tt 26
-- 机器人相关接口 28
12 根据点位 UUID 或外部编码查询机器人 Query robot by node UUID or node foreign code (上游系
统 -> Kuka 系统 ) .ss 28
13 机器人信息查询接口 Query Robots (上游系统 -> Kuka 系统 ) eecccccccccccscscssssesssesesesessesssesssssesees 32

V1.0    05.2025    KUKA AMR                                                                                                            1/49

## Page 2

KUKA

14 下发机器人移动搬运任务 Dispatch Move Carry Task 〔(上游系统 -> Kuka 系统) .ee 34
15 机器人充电 Dispatch Charge Robot Task 〈上游系统 -> Kuka 系统 ) Ne 35
16 入场机器人 Insert The Robot Into The Map 〈上游系统 -> Kuka 系统 ) .ee 36
17 离场机器人 Remove The Robot From The Map 〈上游系统 -> Kuka 系统 ) pp 37
-- AMIR 地图点位与区域相关接 口 eeesseeessecsneesneeesneesusesssessueesnscesueesnsessueesnessusesneeesusesneessneesneeeseeesnsensees 38
18 查询所有 WCS 区域信息 Query All WCS Area Information 〔(上游系统 -> Kuka 系统 ) vee 38
19 区域内点位信息查询 Query Nodes of the Area (上游系统 -> Kuka 系统) .es 39
20 查询点位所属区域 Query Area by Map Node 〈上游系统 -> Kuka 系统 ) .RN 41
21 查询所有禁行区 Query All Forbidden Areas (上游系统 -> Kuka 系统 ) .pp 42
22 查询指定禁行区 Query One Forbidden Area (上游系统 -> Kuka 系统 ) .Rs 44
23 更新指定禁行区的状态 Update Forbidden Area Status (上游系统 -> Kuka 系统) .ee 45
24 查询功能点位 Query Function Node (上游系统 -> Kuka 系统 ) eececccececssssscsesecsssecsescseseesevsvsveeees 47
-- 接口响应报文说明 ee eccccescsesssscscscscscscscscscscscscscscscscscscscscscscssvasssasasasssasssasacscscssssssssasscsescecscscseseeeseenenens 49
版本号          修改内容                         | 昌期 |修改人

2. 接口回调，除了 missionCode，和
missionStatus 都改成非必填

 2.0.0 1.辊简车任务，新增辊简取放完成后“| 20240708 李南

1.2.3               |  1.任务下发增加复合机器人类型 2023.11.22            陈文越

是否通知字段 actionlnform
2.料箱车任务，新增料箱取放完成后
是否通知字段 takeActionlnform

putActionlnform

取放交互一般用于与输送线对接时使

 2.11.1   1. 新增接口， /jobQuery。 20240125 赵禾川

2. 更新 与禁区 文相关 API 以及 API
/missionCancel 的请求参数和请求体示

 2.11.2         ;   1. 新增接口: 20250214 张骞文

/queryRobByNodeUuidOrForeignCode.
2. 更新 API
/updateForbiddenAreaStatus 的请求参
BAS RAN

/queryContainerAll
(可以同时查询入场和离场状态的容

V1.0    05.2025    KUKA AMR                                                                                                            2/49

## Page 3

KUKA

/containerln

新增时可以指定容器校验码)

1. 新增接口: 20250414 AI, A
/robotMoveCarry

2. 新增接口:

/queryFunctionNode

3. 状态回调新增 WAITFEEDBACK 状态

类型〈流程节点等竺放行时回调该状

态)

1. 新增接口: 20250414赵禾川
/insertRobot

2. 新增接口:

/removeRobot

3. 新增接口:

/chargeRobot

-- AMR 任务执行相关接口

1任务下发 Dispatch Mission (上游系统 -> Kuka 系统)

@ ApPl 基本信息
Ah | submitMisson
 PUES TR

APIURL http://[IP:Port]/interfaces/api/amr/submitMission

@ Headers

参数名称          参数值              是否必须

公共请求参数说明

V1.0    05.2025    KUKA AMR                                                                                                            3/49

## Page 4

requestlid
|  missionCode
missionType

viewBoardType

robotType

robotModels

robotids

priority

containerModelCode
containerCode
templateCode
lockRobotAfterFinish

unlockRobotlid

unlockMissionCode

idleNode                      |  String

String
String

String

String

—     py

库存组织 ID(或工
Ie 供应商代

(ea
T      任务类型:
he FE A)
动:RACK_MOVE
辊简车搬运任务:
ROLLER_MOVE
料箱车搬运:
PICKER_MOVE
又车搬运:
FORKLIFT_MOVE
机器人移动:
MOVE
复合机器人搬运:
ROBOTICS_MOVE

PLE Ahh   eee
WHA: LIFT

辊简类: ROLLER
料箱类:PICKER
车: FORKLIFT

加     Site   本

Integer

String
String
String

Boolean

String

String

V1.0    05.2025    KUKA AMR

L5G 级,1-99，
   小， 优先级

一
一
一
天需要流程 结束
后机器人保持人
锁定状态

当前小车的在
F务的锁定状

原则上不允许为
空，如果为空

就会通过通过机
器人具体信号或
容器类型编码进
AT HET

支持指定多个，会
人人  个
适的执行任务;

支持指定多个，会
从多个中指定一个
合适的执行任务;
如果小车前一个任
务流程结束后执行
锁车，该字段必填
批量任务下发时
有效

多用于流程结束
后，机器人放下
容器之后还需要
当前小车搬运的
场景

F             作业流程    完成后，

j于校验解锁是
AEWA

4/49

## Page 5

指定机器人停放区
域/点

oe                  —                                                      当前任务包含的流
程节点信息列表具
体字段参考一下章

如果是调用流程模
板，则次字段可以

1.1 货架移动任务(missionType=RAck_MovE)

@ 请求参数说明
Integer                           序号 默认 1 为起始货
架的起点，  , 后续需
要停留点的需要依次
递增，终点值最大

                     E业路径位置
FE业位置类型，
点位; NODE_POINT
区域: NODE_AREA

jPutpows | Boolean                              作业点位是否需要放
下货架

we                   String                                                                     当前任务点结束后放
了策略:
iz: AUTO
2): MANUAL

waitingMillis          Integer                                   动触发离开当
务节点的时间，默认 | passStrategy
单位:毫秒           是手动则可
不 填 ，
passStrategy
是自动则必
填

@ 请求报文示例
{
"orgid": "UNIVERSAL",
"requestid": "request202309250001",
"missionCode": "mission202309250001",
"missionType": "RACK_MOVE",
"viewBoardType": ""
"robotModels": [
"KMP600I"
1,
"robotlds": [
"44"
1,
"robotType": "LIFT",

V1.0    05.2025    KUKA AMR                                                                                                            5/49

## Page 6

KUKA

"priority": 1,
"containerModelCode": "10001",
"containerCode": "1000002",
"templateCode": "",
"lockRobotAfterFinish": false,
"unlockRobotld": "",
"unlockMissionCode": '"",
"idleNode": "A000000013",
"missionData”: [
{
"sequence": 1,
"position": "M001-A001-45",
"type": "NODE_POINT",
"putDown": false,
"passStrategy": "AUTO",
"waitingMillis": 0
1,

"sequence": 2,
"position": "M001-A001-40",
"type": "NODE_POINT",
"putDown": true,
"passStrategy": "AUTO",
"waitingMillis": 0
}
]
}
@ 请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

}
1.2 4 fal 42 PIS ES (missiontype=ROLLER_MOVE)
@ 请求参数说明

sequence          Integer           T              序号 默认 1 AA
一个节点，后续需要

“position Sting    1

type                String              T                            on
点位;NODE
Xt: NODE

“bincode se SSCSC*SCR I
“rollerievel mteeer | FF OSS MAR | SRE

V1.0    05.2025    KUKA AMR

6/49

## Page 7

actionType                      String

actionConfirm

actionInform

@ 请求报文示例
{
"orgid": "UNIVERSAL",
"requestid": "request202309250002",
"missionCode": "mission202309250002",
"missionType": "ROLLER_MOVE",
"viewBoardType": ""
"robotModels": [
"KMP600I-1-T2"
1],
"robotlds": [
45"
1],
"robotType": "ROLLER",
"priority": 1,
"templateCode":
"lockRobotAfterFinish": false,
"unlockRobotld": ""
"unlockMissionCode":
"idleNode": "4000000013",
"missionData”: [
{
"sequence": 1,
"position": "M001-A001-49",
"type": "NODE_POINT",
"actionType": "ROLLER_RECEIVE",
"binCode": "1000002",
"rollerLevel": 1,
"deviceCode": "bea8f740-5dcc-11ee-91ba-c7b4e0c01550",
"actionConfirm": false,

V1.0    05.2025    KUKA AMR

放

在点位上需要执行的
动作

ROLLER_RECEIVE: 取
ROLLER_SEND: 方

当一 个点位
对应多个设
备时需要传
入该值
当需要对接
输送线且需
要自动上下
料时，需要
传入对应的
执行指令;
如果是人工
上下料或者
是夹取料，
则可以不传

Boolean   pf false      上下料前是否需要确
认

般用于与
输送线对接
交互

Boolean   ae false      上下料后是否需要通
知

般用于与
输送线对接
交互

7/49

## Page 8

KUKA

"actionInform": false
1,
{

"sequence": 2,

"position": "M001-A001-40",

"type": "NODE_POINT",

"actionType": "ROLLER_SEND",

"binCode": "1000002",
"rollerLevel": 1,

"deviceCode": "bea8f740-5dcc-11ee-91ba-c7b4e0c01550",

"actionConfirm": false,

"actionInform": false

}
]
}
请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

}

1.3 BL FB LIS (missionType=PICKER_MOVE)

sequence                        Integer

putActionConfirm

@ KR A
{
"orgld": mu
"requestid": "",

V1.0    05.2025    KUKA AMR

T           搬运料箱序号，从 1
开始，也可以认为是
执行的优先级

TORR ee finite mcf
TR MRI Ae frteke nin
To | | 料箱所在的目标点位
了 |和料箱所在的目标村位

 false     取料箱时之前
要确认

 false     取料箱后是否需

F

—
—
—
—
ee |料箱号

           放料箱后是否需要通

8/49

## Page 9

KUKA

"missionCode":
"missionType": ""
"viewBoardType":
"robotModels": [],
"robotlds": [],
"robotType": ""
"priority": 0,
"containerModelCode": ""
"containerCode": ""
"templateCode": ""
"lockRobotAfterFinish": true,
"unlockRobotld": ""
"unlockMissionCode":
"idleNode": "",
"missionData”: [
{
"sequence": 1,
"binCode": "box-001",
"startPosition": ""
"startSlotCode": a
"takeActionConfirm”': false,
"takeActionInform": false,
"endPosition": ""
"endSlotCode": mw
"putActionConfirm": false,
"putActionInform": false

"sequence": 2,
"binCode": "box-002",
"startPosition": ""
"startSlotCode": a
"takeActionConfirm”': false,
"takeActionInform”"': false,
"endPosition": ""
"endSlotCode": mw
"putActionConfirm": false,
"putActionInform": false
1,
{
"sequence": 3,
"binCode": "box-003",
"startPosition": ""
"startSlotCode": a
"takeActionConfirm”': false,
"takeActionInform”"': false,
"endPosition": ""
"endSlotCode": mw
"putActionConfirm": false,
"putActionInform": false

V1.0       05.2025       KUKA AMR                                                                                                                                                                    9/49

## Page 10

KUKA

}

]

}

@ 请求返回报文示例

{
"data": null,
"code": "0",
"message": null,
"success": true

}

1.4 X 4 RRIS LES (missiontype=FORKLIFT_MOVE)
@ 请求参数说明

sequence          Integer           T              序号 默认 1 为任务第
停留点的需要依次弟
增，终点值最大
|                      |

am Sting ST             fe

meee |       |
数           EYER
要确认

@ 请求报文示例
{

"orgld": "9001",
"requestld": "request2023030800001",
"missionCode": "mission2023030800001",
"missionType": "FORKLIFT_MOVE",
"viewBoardType": "WO01",
"robotType": "FORKLIFT",
"robotModels": [],
"robotlds": [],
"priority": 1,
"containerModelCode": "",

"containerCode": "",
"templateCode": "",
"lockRobotAfterFinish": false,
"unlockRobotld": "",
"unlockMissionCode": '"",
"idleNode": "",
"missionData”: [
{
"sequence": 1,
"position": "",
"stackNumber": 0,

V1.0    05.2025    KUKA AMR                                                                                                           10/49

## Page 11

KUKA

"actionConfirm": false
1,
{

"sequence": 2,
"position": ""
"stackNumber" :0,

"actionConfirm": false

}

]

}

@ 请求返回报文示例

{
"data": null,
"code": "0",
"message": null,
"success": true

}
1.5  机器人移动 (missionType=MovE)

sequence          Integer                           序号 默认 1 为经过的
第一个点， , 后续需
要停留点的需要依次
递增，终点值最大

-Sm | 上|一一  作业路径位置

[si __J                                String                                                                 作业位置类型;
点位; NODE_POINT
区域:， NODE_AREA
passStrategy                 String                           F              AUTO          当前任务点结
行策略;
自动: AUTO
手动: MANUAL

waitingMillis                 Integer                         F                                   动触发离开当前任 | 若
务节点的时间，默认 | passStrategy
单位:毫秒           是手动则可
不 填 ，
passStrategy
是自动则必
填

@ 请求报文示例
{
"orgid": "UNIVERSAL",
"requestld": "request202309250005",
"missionCode": "mission202309250005",
"missionType": "MOVE",
"viewBoardType": ""
"robotModels": [
"KMP600I"

V1.0    05.2025    KUKA AMR                                                                                                           11/49

## Page 12

KUKA

1],
"robotlds": [
"44"
1],
"robotType": "LIFT",
"priority": 1,
"templateCode":
"lockRobotAfterFinish": false,
"unlockRobotld": ""
"unlockMissionCode":
"missionData”: [
{
"sequence": 1,
"position": "M001-A001-26",
"type": "NODE_POINT",
"passStrategy": " MANUAL",
"waitingMillis": 0
1,

"sequence": 2,
"position": "M001-A001-31",
"type": "NODE_POINT",
"passStrategy": " MANUAL",
"waitingMillis": 0
}
]
}
@ 请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

}
1.6 复合机器人任务(missionrype=RoBoTics_MovE)
@     "yaa

er             Integer                                       序号 默认 1 为起始货
架的起点，，后续需
要停留点的需要依次
递增，终点值最大
| Cd
|

Position String          Ht {|  作业路径位置

type                              String                                                                   作业位置类型;
点位， NODE_POINT
区域: NODE_AREA

“applicationName | Sting)  T | CLR
“parame | sting | SON

V1.0    05.2025    KUKA AMR                                                                                                           12/49

## Page 13

KUKA

pRB

—a——     a               AUTO   7 A EAE 结束后放
行策略;
Az: AUTO
手动: MANUAL

waitingMillis         Integer                                    动触发离开当前任
务节点的时间，默认 | passStrategy
单位:毫秒          是手动则可
不 填 ，
passStrategy
是自动则必
填

@ 请求报文示例
{
"orgid": "UNIVERSAL",
"requestid": "request202311220001",
"missionCode": "mission202311220001",
"missionType": "ROBOTICS MOVE",

"viewBoardType":
"robotModels": [
"KMR-IISY"
1],
"robotlds": [
45"
1],
"robotType": "COMPOSITE",
"priority": 1,
"containerModelCode": "10001",
"containerCode": "1000002",
"templateCode": ""
"lockRobotAfterFinish": false,
"unlockRobotld": ""
"unlockMissionCode":
"idleNode": "000000014",
"missionData”: [
{
"sequence": 1,
"position": "M001-A001-45",
"type": "NODE_POINT",
“applicationName ": “app”,
"params": “{\"key1\": \"value1\",\"key2\": \"value2\"}”,
"passStrategy": "AUTO",
"waitingMillis": 0
1,

"sequence": 2,
"position": "M001-A001-40",
"type": "NODE_POINT",
“applicationName ": “app”,
"params": “{\"key1\": \"value1\",\"key2\": \"value2\"}’,

V1.0    05.2025    KUKA AMR                                                                                                           13/49

## Page 14

KUKA

"passStrategy": "AUTO",
"waitingMillis": 0
}
]

}
@ 请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

2 任务取消 Cancel Mission (上游系统 -> Kuka 系统)

@ ApPl 基本信息

 (ssi

APIURL http://[IP:Port]/interfaces/api/amr/missionCancel
  POST

@ Headers

aaa |

@ 请求参数说明

requests | Sting 3 TT) Rid, HEAP eae

加        =
—_         mm | ft   ~

pre frm | RRR |
了

cancelMode            String                      T            FORCE | 取消模式:                        当前

FORCE 强制取消，立即结       关亚
NORMAL 普通取消，不会 | 任
取消小车正在执行的任

务，等待小车把当前任务

执行完，再取消流程
REDIRECT_END 前往终点，      FORCE,
不会取消小车正在执行的 | 是具体还

V1.0    05.2025    KUKA AMR                                                                                                           14/49

## Page 15

任务，等待小车把当前任 | 需要看机
务执行完，如果目标点不 | 器人维度
需要降下货架，则小车携 | 是否支持
带货架前往终点，和否则空 | 取消

车移动至终点
REDIRECT_START 前往起
点，不会取消小车正在执
行的任务，等待小车把当
前任务执行完，如果目标
点不需要降下货架，则小
车携带货架回到起点，和否
则空车移动回到起点

reasm String | IF | |取消原因

@ 请求报文示例
{
"redquestld": "request202309250006",
"missionCode": "mission202309250004",

"containerCode": "",
"position": mu
"cancelMode": "FORCE",

"reason":
}
@ 请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

3 任务放行 Resume Workflow (上游系统 -> Kuka 系统)

@ API 基本信息
 节点任务完成后，上游对任务完成信息的反馈

APIURL http://[IP:Port]/interfaces/api/amr/operationFeedback

@ Headers

 application/ison |是 | |

@ 请求参数说明

 RIG HG wuidse fe)

V1.0    05.2025    KUKA AMR                                                                                                           15/49

## Page 16

‘missionCode [Sting | SSS*d 5
“containercode | Sting) SSS         eo
“position | Sting SSS        当前执行作业的节点

@ 请求报文示例

{
"requestid": "request202309250007",
"containerCode": "",
"missionCode": "mission202309250005",
"position": ""
}
@ 请求返回报文示例
{
"data": null,
"code": "0",

"message": null,
"success": true

}

4作业看板查询接口 Query Jobs (上游系统 -> Kuka 系统)

@ ApPl 基本信息
API 名称                 jobauerry eee
作业看板查询接口

http://[IP:Port]/interfaces/api/amr/jobQuer
HTTP Method

@ Headers

@ 请求参数说明

流程实例 id
状态

机器人编号
流程配置名称

|
|
|
10: 待执行，20: 执行
中; 25: 等待放行;
28: 取消中; 30: 已完
成; 31: 已取消;35:
手动完成， 50: 告警;
60: 流程启动异常
|
|

V1.0    05.2025    KUKA AMR                                                                                                           16/49

## Page 17

KUKA

“worlfiowCode | String «| [FF |  ，|尖本山       一 |
Smaps testngs | LPB         -
“ereateUsername | Sting FI

sourceValue      Integer                     来源
2: FOF AES; 3:
PDA 任务; 4: 设备任
3; 5: MLS 任务;6;
前端界面触发，7: 流
程事件触发;

Integer                       返回作业条数，默认返 |
回 10 条
@ 返回字段说明

 作业及中

eC ER
 suing HS
robots String ES

Integer    FE业状态
0: 待执行， 20: 执行
H; 25: 等竺放行;
8: 取消中; 30: mo
成; 31: GRY:
FE 动完成; 50: ne
0: 流程启动异常

workflowcode | String | WRB |
 AI
“maptode String SEEING
“targetceliCode stme Hine
“begincelicode | String im
“targetCeliCodeForeign | String) Hin RMA
“beginceliCodeForeign sting ASIA LIANG
   ee

warnFlag                     Integer        告警标志
0: 正常，1: 告警
Tie Se AS |
completeTime           String       流 fo    = 时 间 Cyyyy-
MM-dd HH:mm:ss )

spendTime                           Integer          TRE AE Be TA] CRD)            |
_createUsername | String |操作员 |

createTime               String       流程创建时间 (yyyy-
MM-dd HH:mmiss )

String    作业来源
INTERFACE: 接口平台;
PDA: PDA fh 发 ;
DEVICE: 设备触发;
MLS: MLS 触发，SELF，
前端界面触发，EVENT:
流程事件触发;

 物料信息              现场再

V1.0    05.2025    KUKA AMR                                                                                                           17/49

## Page 18

@ 请求报文示例〈仅供参考，根据实际需要填写参数，所有字段非必填)

{
"containerCode": "C001",
"createUsername': "admin",
"jobCode": "T000096284",
"limit": 10,
"maps": ["TEST"],
"robotld": "1",
"sourceValue": 6,
"status": 20,
"targetCellCode": "TEST-1-90",
"workflowCode": " W000000587",
"workflowld": 100218,
"workflowName": "Carry01"

}

@ 请求返回报文示例
{
"data": [
{
"jobCode": "T000096284",
"workflowld": 100218,
"containerCode": "C001",
"robotld":"1",
"status": 20,
"workflowName": "Carry01",
"workflowCode": " W000000587",
"workflowPriority": 1,
"mapCode": "TEST",
"targetCellCode": "TEST-1-90",
"beginCellCode": "TEST-1-80",
"targetCellCodeForeign": "DROPPOINT",
"beginCellCodeForeign": "PICKPOINT",
"finalNodeCode": "TEST-1-90",
"warnFlag": 0，
"warnCode": null
"completeTime": null,
"spendTime": null,
"createUsername": "admin",
"createTime": "2025-01-10 16:01:42",
"source": "SELF",
"materialsinfo": "-"
+L
"code": "0",

"message": null,
"success": true

}

V1.0    05.2025    KUKA AMR                                                                                                           18/49

## Page 19

KUKA

-- AMR 容器相关接口

5 容器入场 Insert Container (上游系统 -> Kuka 系统)

@ ApPl 基本信息
 containerin
 容器相关操作，入声

APIURL http://[IP:Port]/interfaces/api/amr/containerin
  POST

@ Headers

[aa ae |

requestld          String   32   T       OR ids He 等
uuid32 位

containerType                 String                F                 容器类型
货架:，RACK
料箱 :BIN(暂未实
见)
containerModelCode           String                                  容器模型编码
chew-true
时必传

容器

enterOrientation       ie s aT

@ 请求参数说明

                        cana
                         false           TSI Bd A a

a

a ae

-一二-一

a             String                                                            7 RI                      当

isNew=true
时可以指
定是否配
置校验码

withDefaultValidationCode    Boolean                   false | 配置容器默认校验 | 4
ng                                      isNew=true

V1.0    05.2025    KUKA AMR                                                                                                           19/49

## Page 20

KUKA

@ 请求报文示例〈当前货架入场)
{

"requestid": "request202309250008",
"containerType": ""
"containerCode": "10",

"position": "M001-A001-31",
"containerModelCode": ""
"enterOrientation": ""

"isNew": false

}

St

@ 请求报文示例《货架新增并入场,不配置校验码)

{

"requestid": "request202309250009",
"containerType": ""
"containerCode": "10",

"position": "M001-A001-31",
"containerModelCode": "model1",
"enterOrientation": ""
"isNew": true,
"containerValidationCode":
"withDefaultValidationCode": false

}

@ 请求报文示例《货架新增并入场,配置指定校验码)

{

"redquestld": "request202309250010",
"containerType": ""
"containerCode": "10",

"position": "M001-A001-31",
"containerModelCode": "model1",
"enterOrientation": ""

"isNew": true,
"containerValidationCode": "ID:10",
"withDefaultValidationCode": false

}

@ RCN BATHE AT AC RU BS

St

{

"requestid": "request202309250011",
"containerType": ""
"containerCode": "10",

"position": "M001-A001-31",
"containerModelCode": "model1",
"enterOrientation": ""
"isNew": true,
"containerValidationCode":
"withDefaultValidationCode": true

}

@ 请求返回报文示例

V1.0    05.2025    KUKA AMR

20/49

## Page 21

KUKA

"data": null,
"code": "oO",
"message": null,
"success": true

}

6 容器出场 Remove Container (上游系统 -> Kuka AA)

@ API 基本信息
 containerOut
 sec, a

APIURL http://[IP:Port]/interfaces/api/amr/containerOut

HTTP Method                POST

@ Headers

参数名称         参数值             是否必用
Content-Type       application/json

@ 请求参数说明

@ 请求报文示例

{
"requestid": "request202309250009",
"containerType": "",
"containerCode": "10",
"position": "M001-A001-31",
"isDelete": false
}
@ 请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true
}

V1.0    05.2025    KUKA AMR

5

货架: RACK
BLA: BING AS
| 容器编号
| 容器出场位置
 Tea sa

21/49

## Page 22

KUKA

7 容器信息更新 Update Container (AN MH: 位置和空满状态的更
新) (上游系统 -> Kuka 系统)

@ ApPl 基本信息

 守信和更新

APIURL http://[IP:Port]/interfaces/api/amr/updateContainer
  POST

@ Headers

@ 请求参数说明

货架: BUCKET
料箱:BIN(暂不支持)

Joc

@ KRABI
{
"requestid": "request202309250010",
"containerType": "BUCKET",
"containerCode": "10",
"originPosition": "M001-A001-31",
"targetPosition": "M001-A001-30",
"emptyStatus": "EMPTY",
"reason": ""
}
@ 请求返回报文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

}

V1.0    05.2025    KUKA AMR

22/49

## Page 23

KUKA

8接口回调 Mission Status Callback (Kuka 系统 -> 上游系统)

@ ApPl 基本信息

 党间作

APIURL http://[IP:Port]/interfaces/api/amr/missionStateCallback
  POST

@ 请求参数说明

mmcde Sting 3 T yd |
‘viewBoardType se | FF | 人sk
“containercode sme FF Aha
merge [sting [fF a sn

                       Br FEA

message
ea |

@ 请求报文示例
{
"missionCode": "mission202309250005",

IF | |当
a ee
String                                T                                 作业当前状态

开始移动: MOVE_BEGIN
到达任务节点;ARRIVED
i 升 完 成 :
UP_CONTAINER
放 下完成 :
DOWN_CONTAINER
辊 ff 上 料    :
ROLLER_RECEIVE
 fl 下 -    :
ROLLER_SE
料 8 取 料    :
PICKER_RECEIVE
料箱下料完成:
PICKER_SEND
又车又取完成
FORK_UP
叉车放下完成;
FORK_DOWN
等竺放行， WAITFEEDBACK
任务完成: COMPLETED
任务取消完成: CANCELED

message   fet aor

HE EARLE th

"viewBoardType":
"slotCode": "",
"robotld": "14",
"containerCode": "1000002",
"currentPosition": "M001-A001-31",
"missionStatus": "MOVE_BEGIN",

V1.0    05.2025    KUKA AMR

23/49

## Page 24

KUKA

"message": "",
"missionData": {}

}

9 容器模型查询接口 Query Container Model Codes (上游系统 -> Kuka 系
统)

@ ApPl 基本信息

 查询所有已配置容器模型

APIURL http://[IP:Port]/interfaces/api/amr/queryAllContainerModelCode

@ 无请求参数
@ 请求返回报文示例
{
"code": "0",
"message": null,
"success": true,
"data": [
"10001"
]
}

10 查询容器模型推荐的可存放区域 Query Area Codes By Conatiner
Model (上游系统 -> Kuka 系统)

@ ApPl 基本信息

 queryAreaCodeForContainerModel
 查询容器模型推荐的可存放区域

APIURL http://[IP:Port]/interfaces/api/amr/queryAreaCodeForContainerModel
 GET

@ 请求参数说明
| 容器模型编码              |

containerModelCode

Te
noContainerFirst      Boolean             F        false | 是否区域内无容器优先，
ture 是
false &

@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/queryAreaCodeForContainerModel?containerModelCode=10001&
noContainerFirst=false

V1.0    05.2025    KUKA AMR                                                                                                           24/49

## Page 25

KUKA

@ 请求返回报文示例
{
"code": "0",
"message": "",
"success": true,
"data": [
"areaCode1",
"areaCode2",
"areaCode3"
]
}

11-1 容器信息查询接口(仅查询入场状态容器) Query Containers (上游
系统 -> Kuka 系统)

@ API 基本信息
 containerQuery
 容器信息查询接口(仅查询入场状态的容器)

APIURL http://[IP:Port]/interfaces/api/amr/containerQuery
HTTP Method

@ Headers

参数名称          参数值              是否必须

© 请求参数说明

gen

nodeCode
 string

|
|
|
|
emptyFullStatus            pm]

@ 返回字段说明

containerCode   容器编码

点位编码
容器模型编码

—
—
|
—
  容器的空满状态;

 容器编四
| |
| |

20
满1
全部2

nodeCode  点位编码                   |
Integer         入场/离场状态
0-离场，1-入场

containerModelCode 容器模型

V1.0    05.2025    KUKA AMR                                                                                                           25/49

## Page 26

KUKA

emptyFullStatus
isCarry                      Integer       搬运状态 0-静止 ，1-搬
运中
String           a 半角度
String | 容器校验码 |
mapCode                                            地图 人
| districtCode          | String     | 地图片区编码            |

orientation
containerCheckCode

@ 请求报文示例〈查询指定容器)
{
"nodeCode": "M001-A001-30",
"containerModelCode": "10001",
"containerCode": "10",
"areaCode": "A000000014",
eee 2

e aR 回报文示例

{
"data": [
{

"containerCode": "10",
"nodeCode": ""
"orientation": "0.0",

"containerModelCode": "10001",

"emptyFullStatus": 0,

"inMapStatus": 1,

"isCarry": 0,

"containerCheckCode": "10",
"mapCode": "ABC",
"districtCode": "1"

}
1],
"code": "0",

"message": null,
"success": true

11-2 容器信息查询接口(同时查询入场和离场状态容器) Query
Containers (上游系统 -> Kuka 系统)

@ API 基本信息
API 描述

[APIURL i

全一

@ Headers

containerQueryAll
容器信息查询接口(同时查询入场和离场状态的容器)

http://[IP:Port]/interfaces/api/amr/containerQueryAll                      |
POST

V1.0    05.2025    KUKA AMR                                                                                                           26/49

## Page 27

参数名称 — | Set     是否必须
 spplicatonfson | 是 | |

_

@ 请求参数说明

containerCode
 String
 String

emptyFullStatus                                                  容器的空满状态:
20
满1
                               容器入场离场状态
离场0
入场1

Integer         入场/离场状态
0-离场，1-入场
_containerModelCode _

@ 返回字段说明

containerModelCode | String           容器模型

emptyFullStatus   0-空 1-满

 这 me me
运中

 EAI

“containercheckCode | sting | HME ，
mapcode | Sting | SMS
“distritCode | Sting | SIRS

@ 请求报文示例〈查询指定地图所有容器)

{
"mapCode": "ABC"
}
@ 请求返回报文示例
{
"data": [
{

"containerCode": "10",
"nodeCode": "1-1",
"orientation": "0.0",
"containerModelCode": "10001",
"emptyFullStatus": 0,

V1.0    05.2025    KUKA AMR                                                                                                           27/49

## Page 28

KUKA

"isCarry": 0,
"containerCheckCode": "10",
"mapCode": "ABC",
"districtCode": "1"

"containerCode": "20",
"nodeCode": "1-1",
"orientation": "0.0",
"containerModelCode": "10001",
"emptyFullStatus": 0,

"isCarry": 0,
"containerCheckCode": "20",
"mapCode": "ABC",
"districtCode": "1"

}

1],

"code":  "oO",
"message": null,
"success": true

-- 机器人相关接口

12 根据点位 UUID 或外部编码查询机器人 Query robot by node UUID or
node foreign code (上游系统 -> Kuka 系统)

@ ApPl 基本信息

API 名称                   queryRobByNodeUuidOrForeignCode

API 描述                   根据点位 UUID 或点位外部编码查询机器人

APIURL http://[IP:Port]/interfaces/api/amr/queryRobByNodeUuidOrForeignCode

@ Headers

是        和

Content-Type          application/json

@ 请求参数说明

V1.0    05.2025    KUKA AMR                                                                                                           28/49

## Page 29

KUKA

PoP yy

@ 返回字段说明

点位 UUID 或点位外部编
码

  机器人编码               |
  机器人型号               |

mapCode
floorNumber
buildingCode
containerCode

occupyStatus

batteryLevel
nodeCode

robotOrientation
missionCode

liftStatus
reliability

runTime

karOsVersion

mileage
leftMotorTemperature
rightMotorTemperature
liftMotorTemperature
rotateMotorTemperature
rotateTimes

liftTimes
nodeForeignCode

V1.0    05.2025    KUKA AMR

 地图编码
文乡

  工厂或者仓库编码
  机器人持有容器的编码

 1-离 场 ，2-离 线 ，3-空
As 4-任务中;， 5-充电
中;，6-更新中，7-噶常

 是否占用: OA; 1-
占用

  量〈单位，百分比)

String     [器人当前所处坐标
   单位: 毫米)

    L       当前所处坐标
y(4     毫米)

 机器人项升状态 CA: 项
Ft; O: 未顶升)

Integer     定位置信度: 0-不可信;

 运行时长

  件

 片区编码

wn
o
a
5
oa

String | 右电机温度

String | 机器人所在节点的外部
编码

wn
o
a
5
oa

任务号

EEE
当使用节
点的外部
编码查询
机器人
时，该字
BORA
值。

29/49

## Page 30

KUKA

@ 请求报文示例
1) 示例1
{
"nodeCode": "startNode1, startNode2"
}

2) 示例 2
{

"nodeCode": "test01-5-91"
}

@ 返回报文示例
1) 示例1
{
"data": [
{
"robotld": "925",
"robotType": "KMP600I",
"mapCode": "test01",
"floorNumber": "5",
"buildingCode": "test",
"containerCode": "",
"status": 3,
"occupyStatus": 0，
"batteryLevel": 1,
"nodeCode": "test01-5-64",
"x": "3000.0",
"vy": "15000.0",
"robotOrientation": "90.0",
"missionCode": null,
"liftStatus": 0，

"reliability ": 1,
"runtime ": "6745",
"karOsVersion ":"",

"mileage ": "0.0",
"leftMotorTemperature ": "0.0",
"rightMotorTemperature ": "0.0",
"liftMotorTemperature ": "0.0",
"rotateMotorTemperature ": "0.0",
"rotateTimes ": O,

"liftTimes ": O,

"nodeForeignCode": " startNode1"

"robotld": "80211",
"robotType": "KMP600I",
"mapCode": "test01",
"floorNumber": "5",
"buildingCode": "test",
"containerCode": "",
"status": 3,

V1.0    05.2025    KUKA AMR                                                                                                           30/49

## Page 31

KUKA

"occupyStatus": 0，
"batteryLevel": 1,
"nodeCode": "test01-5-74",
"x": "1200.0",

"y": "2400.0",
"robotOrientation": "0.0",
"missionCode": null,
"liftStatus": 0，

"reliability ": 1,
"runTime ": "null",
"karOsVersion ":"",

"mileage ": "0.0",
"leftMotorTemperature ": "0.0",
"rightMotorTemperature ": "0.0",
"liftMotorTemperature ": "0.0",
"rotateMotorTemperature ": "0.0",
"rotateTimes ": O,
"liftTimes ": O,
"nodeForeignCode": " startNode2"
}

1,

"code": "0",

"message": null,

"success": true

2) 示例2
{
"data": [
{

"robotld": "2041",
"robotType": "KMP600I",
"mapCode": "test01",
"floorNumber": "5",
"buildingCode": "test",
"containerCode": "",
"status": 3,
"occupyStatus": 0，
"batteryLevel": 1,
"nodeCode": "test01-5-91",
"x": "13000.0",
"y": "30000.0",
"robotOrientation": "-90.0",
"missionCode": null,
"liftStatus": 0，

"reliability ": 1,
"runTime ": "6745",
"karOsVersion ":"",

"mileage ": "0.0",
"leftMotorTemperature ": "0.0",
"rightMotorTemperature ": "0.0",

V1.0    05.2025    KUKA AMR                                                                                                           31/49

## Page 32

KUKA

"liftMotorTemperature ": "0.0",
"rotateMotorTemperature ": "0.0",
"rotateTimes ": O,
"liftTimes ": O,
"nodeForeignCode": ""
}
1,
"code": "0",
"message": null,
"success": true

13 机器人信息查询接口 Query Robots (上游系统 -> Kuka 系统)

@ API 基本信息
API和称 robotauevy
 机器人信息查询接口(蒜认查所有)

APIURL http://[IP:Port]/interfaces/api/amr/robotQuery

@ Headers

参数名称   参数值    是否必须

robotd lstrnge | IF | |机器人编码           1) 如果参
            号              六
      地图编码〈禁有    | 空，则默认

F

@ 请求参数说明

mapCode                          String
果为空)        碍全部。

|
-                                    Te        7
floorNumber       String                        og ( 48 F        2) 请求参
数 mapCode
和
floorNumber
必须一起传
ih.

@ 返回字段说明

  机器人纲码

 机器人型号
 SFE

V1.0    05.2025    KUKA AMR                                                                                                           32/49

## Page 33

KUKA

floorNumber                        String           片区编码

buildingCode                 String        工广或者仓库编码

containerCode                    String          机器人持有容器的编码

status                         Integer       1-离 场 ，2-离 线 ; 3-空
PAs 4-任务中;， 5-充电
中;，6-更新中，7-异常

occupyStatus                 Integer       是否占用: 0-未占用; 1

上

batteryLevel                   Double       Hie (AAT: 百分比)

nodeCode                            String            当前点位

X                     String      机器人当前所处坐标
x(单位: 毫米)

Y                     String      机器人当前所处坐标
y(单位:  毫米)

robotOrientation                  String           机器人当前角度

missionCode                        String            当前任务号

liftStatus                      Integer       机器人项升状态 C1: Tit
升; 0: 未顶升)

reliability                  Integer      定位置信度: 0-不可信，
1-可信

runTime                      String        运行时长

karOsVersion                String        软件版本

mileage                         String         运行总里程

leftMotorTemperature       String        左电机温度

rightMotorTemperature     String        右电机温度

liftMotorTemperature          String           顶升电机温度

rotateMotorTemperature | String           ees WL

rotateTimes                  Integer       托盘旋转次数

liftTimes                      Integer       托盘顶升次数

@ 请求报文示例〈当前货架入场)

V1.0

{

"floorNumber": "A001",
"mapCode": "M001",
"robotld": "13",
"robotType": "KMP600I'

}

请求返回报文示例

{

"data": [

{

"robotld": "13",
"robotType": "KMP600I",
"mapCode": "M001",
"floorNumber": "A001",
"buildingCode": "W001",
"containerCode": "",
"status": 3,
"occupyStatus": 0，
"batteryLevel": 1,
"nodeCode": "46",

05.2025    KUKA AMR

上游下发
任务号

33/49

## Page 34

KUKA

"x": "3000.0",

"vy": "15000.0",
"robotOrientation": "90.0",
"missionCode": null,
"liftStatus": O,

"reliability ": 1,
"runtime ": "6745",
"karOsVersion ":"",

"mileage ": "0.0",
"leftMotorTemperature ": "0.0",
"rightMotorTemperature ": "0.0",
"liftMotorTemperature ": "0.0",
"rotateMotorTemperature ": "0.0",
"rotateTimes ": O,
"liftTimes ": 0
}

1,

"code": "0",

"message": null,

"success": true

14 下发机器人移动搬运任务 Dispatch Move Carry Task (上游系统 ->
Kuka 系统)

@ ApPl 基本信息

robotMoveCarr
下发机器人移动搬运任务，只在任务列表展示，作业看板不可见。

Dispatch a move carry task to the robot. The function is the same as

which in the Map Monitor, directly dispatched to the Mission Manager,
only displayed in the Mission List, and not visible on the Job Board.
http://[IP:Port]/interfaces/api/amr/robotMoveCarry

HTTP Method

@ Headers

 applicationfison |是 | |

a

@ 请求参数说明

| 机器人编号

目标点位编码〈点位
UUID )

a ae
_containerCode | String | TO 容器编号
|

targetNodeCode

V1.0    05.2025    KUKA AMR                                                                                                           34/49

## Page 35

missionCode                                任务号，下发给 mission
manager的任务号。不传
会自动生成。

@ 返回字段说明
下发成功时，返回任务号

@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/robotMoveCarr
{
"robotld": "10001",
"containerCode": "C0002",
"targetNodeCode": "TEST-1-45"

}
@ 返回报文示例
示例1
{
"data": "I-CARRY-1744609148201",
"code": "0",

"message": "OK",
"success": true

}

15 机器人充电 Dispatch Charge Robot Task (上游系统 -> Kuka 系统)

@ ApPl 基本信息

API名称 | chargeRobot
机器人充电，只在任务列表展示，作业看板不可见。

Dispatch charge task to the robot. The function is the same as which in

the Map Monitor, directly dispatched to the Mission Manager, only
displayed in the Mission List, and not visible on the Job Board.

http://[IP:Port]/interfaces/api/amr/chargeRobot
POT

HTTP Method                POST

@ Headers

 applicationfison |是 | |

@ 请求参数说明

| 机器人编号

Te                                          |
necessary                       Integer                          T                             是否强制生成充电任务
1-一定生成任务并排

V1.0    05.2025    KUKA AMR                                                                                                           35/49

## Page 36

BA; 0-低 于 lowestLevel
则去充电，   无资源的情

可 直接完成

“targetlevel finger] STS,

lowestLevel                 Integer                                                WEF 电的电量，单位
=F Coe
必传
== ED
wes 自动生成

@ 返回字段说明

无

@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/robotMoveCarr
{

"lowestLevel": 5,
"necessary": O,
"robotid": "2",
"targetLevel": 90

}
@ 返回报文示例
{

"data": null,
"code": "oO",
"message": "OK",
"success": true

16 入场机器人 Insert The Robot Into The Map (上游系统 -> Kuka 系统)

@ API 基本信息

 ingertRobot
入场机器人，该接口目前只文持真车。
Insert the robot into the map. Only support real AMR

currently (distinguished from simulated AMR).

APIURL http://[IP:Port]/interfaces/api/amr/insertRobot
  POST

@ Headers

Content-Type       application/json   SS

@ 请求参数说明

V1.0    05.2025    KUKA AMR                                                                                                           36/49

## Page 37

robotd | String TT 机器人编号               |
jcelcode __} String | fr} BBA AM 位       |

@ 返回字段说明

无

@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/insertRobot
{

"cellCode": "TEST-1-69",
"robotid": "2",
"synchroContainer": 1

}
@ 返回报文示例
{

"data": null,
"code": "oO",
"message": "OK",
"success": true

}

17 离场机器人 Remove The Robot From The Map (上游系统 -> Kuka 系
统)

@ API 基本信息

API 描述      离场机器人
Remove the robot from the map

APIURL http://[IP:Port]/interfaces/api/amr/removeRobot

@ Headers

Temes [acco |

fn

@ 请求参数说明

ee |sme | ft}  机器人编号

@ 返回字段说明

V1.0    05.2025    KUKA AMR                                                                                                           37/49

## Page 38

KUKA

无
@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/removeRobot
{
"robotld": "2",

"withContainer": 1

}
@ 返回报文示例
{
"data": null,
"code": "0",
"message": "OK",
"success": true

}

-- AMR 地图点位与区域相关接口

18 查询所有 Wocs 区域信息 Query All WCS Area Information (上游系统
-> Kuka 系统)

@ ApPl 基本信息

  查询所有 WCS 区域信息 Query All WCS Area Information

APIURL http://[IP:Port]/interfaces/api/amr/areaQuery

@ Headers

aaa | 一

 nL
 pc

|
|
areaType                Integer       区 TE 区  3c fF
5监管区 7又车搬运

V1.0    05.2025    KUKA AMR                                                                                                           38/49

## Page 39

KUKA

@ 请求报文示例
@ 请求返回报文示例

{
"data": [
{
"areaName": "container storage area",
"areaType": 1,
"areaCode": "1-song-1732701220548"
{
"areaName": "temporary storage area",
"areaType": 3,
"areaCode": "1-song-1732701269477"
{
"areaName'": "buffer area",
"areaType": 4,
"areaCode": "1-song-1732701316971"
{
"areaName": "dense storage area",
"areaType": 5,
"areaCode": "1-song-1732701347592"
{
"areaName'": "exclusive area",
"areaType": 6,
"areaCode": "1-song-1732701422092"
{
"areaName'": "fork area",
"areaType": 7,
"areaCode": "1-song-1732701459113"
}
1]
"code": "0",

"message": null,
"success": true

19 区域内点位信息查询 Query Nodes of the Area (上游系统 -> Kuka K

统)

@ ApPl 基本信息

api 名称               | areaNodesQuery
API 描述                区域内点位 (外部编码) 信息查询 Query Nodes of the Area

V1.0    05.2025    KUKA AMR

39/49

## Page 40

KUKA

APIURL http://[IP:Port]/interfaces/api/amr/areaNodesQuery

HTTP Method                POST

@ Headers

Content-Type       application/json   SS

@ 请求参数说明

areacodes | listcSting> | TT) LRU       _—
@ 返回字段说明

 ra        Po

List<String> | 点位外部编码的集合。
若该区域内所有点位均
无外部编码，则该字段
可能为空

@ 请求报文示例

1) 示例1
{
"areaCodes": ["A000000013","A000000014"]
}
2) 示例2
{
"areaCodes": ["1-song-1732612877805"]
}

@ 返回报文示例
1) 示例1
{
"data": [
{
"areaCode": "A000000014",
"nodeList”: [
"97"
]
},

{
"areaCode": "A000000013",

"nodeList”: [
"46",
"So"

]

V1.0    05.2025    KUKA AMR

40/49

## Page 41

KUKA

1],

"code":  "oO",
"message": null,
"success": true

}

3) 示例 2，区域内点位均无外部编码
{
"data": [
{
"areaCode": "1-song-1732612877805",
"nodeList”: [

1],

"code":  "oO",
"message": null,
"success": true

20 查询点位所属区域 Query Area by Map Node (上游系统 -> Kuka 系
统)

@ ApPl 基本信息

  查询点位所属区域 Query Area by Map Node

APIURL http://[IP:Port]/interfaces/api/amr/queryWCSAreaByMapNode

@ Headers

| content-type | application/json |是 | | Cd

nodeuud | String, | IT | |点位uup
@ 返回字段说明

@ 请求参数说明

areaCode                          String             区域编码
area code

V1.0    05.2025    KUKA AMR                                                                                                           41/49

## Page 42

区域可放置的容器的模 | 不一定有
型编码

@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/queryWCSAreaByMapNode?nodeUuid= 1-song-44

@ 返回报文示例
1) 查询到点位所属区域，区域未配置可放置的容器模型编码
{

"data": {
"areaCode": "1-song-1732612831825",
"containerModelCode": ""

1,

"code": "0",

"message": null,

"success": true

}
2) 查询到点位所属区域，区域未配置可放置的容器模型编码
{

"data": {

"areaCode": "1-song-1732612877805",
"containerModelCode": "600i"
1,
"code": "0",
"message": null,
"success": true

}
3) 未查询到点位所属区域
{
"data": null,

"code": "100001",
"message": "No such node in the graph.[7788]",
"success': false

21 查询所有禁行区 Query All Forbidden Areas (上游系统 -> Kuka 系统)

@ API 基本信息
API 名称                   queryAllForbiddenAreas
API 描述                     查询所有禁行区 Query All Forbidden Areas

API URL                    http://[IP:Port]/interfaces/api/amr/queryAllForbiddenAreas
| HTTP Method      | GET

V1.0    05.2025    KUKA AMR                                                                                                           42/49

## Page 43

KUKA

@ Headers

Content-type | applicationvison |是 | |

@ 请求参数说明
无

@ 返回字段说明
forbiddenAread integer | 全id
 SF Oa        _

floorNumber
  禁行区描述

forbiddenAreaStrategy | Integer          禁行策略:0-默认;1-人允许
有车
Integer        状态:0-禁用;1-启 A;
效中;3-失效中
enableType                 Integer        E效方式:0-立即生效;
按给定时间段生效

@ 请求报文示例
无

@ 请求返回报文示例
{
"data": [
{
"forbiddenAreald”": 3,
"mapCode": "1",

"floorNumber": "song",
"description": "",
"forbiddenAreaStrategy": 0，
"status": 1,

"enableType": 0

了

"forbiddenAreald": 4,
"mapCode": "1",
"floorNumber": "1",
"description": "",
"forbiddenAreaStrategy": 0，
"status": 1,

"enableType": 0

了

{
"forbiddenAreald": 2,

"mapCode": mq"

V1.0    05.2025    KUKA AMR                                                                                                           43/49

## Page 44

KUKA

"floorNumber": "song",
"description": "forbidden areal of map song",
"forbiddenAreaStrategy": 0，
"status": 1,
"enableType": 0
}
]，
"code": "0",
"message": null,
"success": true
+],
"code": "0",
"message": null,
"success": true

22 查询指定禁行区 Query One Forbidden Area (上游系统 -> Kuka K
统)

@ API 基本信息
  queryOneForbiddenArea

  查询指定禁行区 Query One Forbidden Area

PPorUntracesyapyamyatenOnetorhdeneea |  http://[IP:Port]/interfaces/api/amr/queryOneForbiddenArea

HTTP Method                GET

@ Headers

forbiddenAreald          Integer                                            PEAT IX id
forbidden area ID
@ 返回字段说明

字段名称  字段描述                        项目用途
型

forbiddenAreald         Integer MTR ig
 地图片区编码

@ 请求参数说明 _

floorNumber        sme RE OSCC‘“‘~*~“—s*~*~S~S~S
 27 DCT          an

forbiddenAreaStrategy   禁行策略:0-默认;1-人允许
有车

V1.0    05.2025    KUKA AMR                                                                                                           44/49

## Page 45

Integer     状态:0-禁用;1-启 A;
效中;3-失效中
enableType             Integer     生效方式:0-立即生效;
按给定时间段生效

@ 请求 URL 示例
http://[IP:Port]/interfaces/api/amr/queryOneForbiddenArea?forbiddenAreald=2

@ 返回报文示例

示例1
{
"data": {
"forbiddenAreald": 2,
"mapCode": "1",

"floorNumber": "song",
"description": "forbidden areal of map song",
"forbiddenAreaStrategy": 0,
"status": 1,
"enableType": 0
1,
"code": "0",
"message": null,
"success": true

23 更新指定禁行区的状态 Update Forbidden Area Status (上游系统 ->
Kuka 系统)

@  Pl 基本信息

 updateForbiddenAreastatus
  更新指定禁行区的状态 Update Forbidden Area Status

APIURL http://[IP:Port]/interfaces/api/amr/updateForbiddenAreaStatus
HTTP Method                POST

@ Headers

@ 请求参数说明

forbiddenAreald          a et a                                                                              在请求体中，

forbiddenAreald 或
forbiddenAreaCode
只能选择其中一

V1.0    05.2025    KUKA AMR                                                                                                           45/49

## Page 46

个作为参数。

status                                String

@ 返回字段说明
无
@ 请求报文示例 request body
1) 示例1
{
"forbiddenAreald": 2,
"status": "0"
}
2) 示例2
{

"forbiddenAreaCode": "map1-01-1739427001143",
"status": "0"

}

@ 返回报文示例 response body
1) 状态修改成功后返回的报文
{
"data": null,
"code": "0",
"message": null,
"success": true
}
2) 状态修改失败后返回的报文
错误原因: 该禁行区的状态已经为0〈禁用) ，仍旧修改其状态为0，则会收到如下错误。如果状态已
经为1〈启用) ，再次修改其状态为1，也会收到类似错误。
{
"data": null,
"code": "100001",
"message": "RCS returns error[[D]com.kuka.rcs.bd.common.grpc.util.CommonGrpcErrorCode.16[can not
update [ MAP_ZONE_STATUS_DISABLE ]]]",
"success": false
}
3) 传参错误
错误原因: 每次只能选择 forbiddenAreald 和 forbiddenAreaCode 中的一个作为请求参数。
{

"data": null,
"code": "100001",

"message": "only one of area ID or area code can be filled in",
"success": false

V1.0    05.2025    KUKA AMR                                                                                                           46/49

## Page 47

KUKA

24 查询功能点位 Query Function Node (上游系统 -> Kuka 系统)

@ APL 基本信息
   queryFunctionNode
  查询功能点位 Query Function Node

APIURL http://[IP:Port]/interfaces/api/amr/queryFunctionNode
  POST

@ Headers

| content-type | application/json |是 | | Cd

© 请求参数说明

functionType                   T                              功能点位类型

                    nae
                    ae

V1.0    05.2025    KUKA AMR                                                                                                           47/49

## Page 48

robotTypeClass | String | FO | |机器人类型编码
containerModel           Strng | IF |容器模型编码

@ 返回字段说明

| nodeCode         | String
| externalCode

|  String
functionType                  Integer

| 点位编码 CUUID)    |
| 点位外部编码     |     |

功能点位类型

containerModelCode

Bh DL SC HF BA A eh eA Sh
码

robotTypeCode                  List<String> | 点位支持的机器人类型
|                           编码

@ 请求报文示例
{

List<String>

"floorNumber": "1",
"functionType": 1,
"mapCode": "TEST"

@ 返回报文示例
{
"data": [
{
"nodeCode": "TEST-1-53",
"externalCode": "",
"functionType": 1,
"containerModelCode": [
"model1"
1,
"robotTypeCode": [
"KMP600I"
]
},
{
"nodeCode": "TEST-1-1",
"externalCode": "",
"functionType": 1,
"containerModelCode": [],
"robotTypeCode": []
}
1,
"code": "0",
"message": "lang.wcs.common.ok",
"success": true

}

V1.0    05.2025    KUKA AMR                                                                                                           48/49

## Page 49

KUKA

-- 接口响应报文说明

@ 成功响应
{

"code": "oO",
"message": mu
"success": true,

"data": wit

}
@ FMA py
{
"code": mn //异常代码
"message": " //Re it fa
"success": false,
"data": null

}
异常信息说明:

An
was
PRIA
eet

cer
BARRIS
酸砚是

V1.0    05.2025    KUKA AMR

49/49