

## Page 1

KUKA

Kuka-interface 標準接口
目錄

Kuka-interface 標準接口 .4 1
-- AMR 任務執行相關接口 ecseeesseeesneesssecsneesssecsneesneessnecsusesnessneesneesasessueesneesnecsneesusecseesusesneessnseeneensnetens 3
1任務下發 Dispatch Mission (上游系統 -> Kuka 系統 ) ceececcsescscscscscscscscscsescscsescscsesesesesesesscsesesesesesesceeees 3
公共請求參數說明 .4 3
1.1 貨架移動任務(missionType=RACK_MOVE) ....ccccccccsescsccscsscscscscscssscssescessessescessesusssessssuessesessuessesaeens 5
1.2 輥簡車搬運任務(missionType=ROLLER_MOVE) ......cccssescssesscsssssescsscscsscssesesessecsesenssestsacssstsasssansneees 6
1.3 料箱搬運(missionType=PICKER_MOVE) 8
1.4 叉車搬運任務(missionType=FORKLIFT_MOVE) ....scscsccsessssessescscsscscsssscsessssscsscsssesscscsssestssessseseceaes 10
1.5 機器人移動(missionType=MOVE) u.....cssecescecceccecsceccscsescscscscseevevavavacacacacecscecscsceceseseseseststststseevens 11
1.6 復合機器人任務(missionType=ROBOTICS_MOVE) Ne 12
2 任務取消 Cancel Mission (上游系統 -> Kuka 系統 ) eececescesccsesseesecseeseeseesesseaseaseaeatesteateseeseesseseeseeses 14
3 任務放行 Resume Workflow (EV RA -> Kuka 系統 ) cecccccccsssscssesssseesesssecsesessecsescsessescseessseeseescaes 15
4作業看板查詢接口 Query Jobs (上游系統 -> Kuka 系統 ) cece cess cscscsescsseseescssssseeeeesseseeaes 16
-- AMIR 容器相關接口 eesseecsneessseesneesssccsneesssecsneesusessnessusesnsessuessnsesnsessueesusessueesusesneesueesneessueesnteeseeesneenees 19
5 容器入場 Insert Container (上游系統 -> Kuka 系統 ) ceeeeccccscsssssssesescssssesceecsescsscsesceseseeneseeeeeaeeneaeeeees 19
6 容器出場 Remove Container (上游系統 -> Kuka 系統 ) ceececcsccssscssessscssesescessscssesescesesesesssceseseseeseseaee 21
7 容器信息更新 Update Container (暫時支持: 位置和空滿狀態的更新) 〈上游系統 -> Kuka 系
統 ) cecssssssesseessecsssesueesuessvecssecunesueesucesnecsesseesucesuessuesasecaesusesueesuessuesueesuessuetsesseesueesusesueeseeatesutesneesneesseesnetes 22
8 接口回調 Mission Status Callback (Kuka 系統 -> 上游系統 ) cceccccscscscsssscscsceceesescsescsceesesesesesteeeeneees 23
9 容器模型查詢接口 Query Container Model Codes (上游系統 -> Kuka 系統) eee 24
10 查詢容器模型推薦的可存放區域 Query Area Codes By Conatiner Model (上游系統 -> Kuka 系
統 ) cecssssssesseessecsssesueesuessvecssecunesueesucesnecsesseesucesuessuesasecaesusesueesuessuesueesuessuetsesseesueesusesueeseeatesutesneesneesseesnetes 24
11-1 容器信息查詢接口(僅查詢入場狀態容器) Query Containers (上游系統 -> Kuka 系統) eee. 25
11-2 容器信息查詢接口(同時查詢入場和離場狀態容器) Query Containers (上游系統 -> Kuka 系統)
tt 26
-- 機器人相關接口 28
12 根據點位 UUID 或外部編碼查詢機器人 Query robot by node UUID or node foreign code (上游系
統 -> Kuka 系統 ) .ss 28
13 機器人信息查詢接口 Query Robots (上游系統 -> Kuka 系統 ) eecccccccccccscscssssesssesesesessesssesssssesees 32

V1.0    05.2025    KUKA AMR                                                                                                            1/49

## Page 2

KUKA

14 下發機器人移動搬運任務 Dispatch Move Carry Task 〔(上游系統 -> Kuka 系統) .ee 34
15 機器人充電 Dispatch Charge Robot Task 〈上游系統 -> Kuka 系統 ) Ne 35
16 入場機器人 Insert The Robot Into The Map 〈上游系統 -> Kuka 系統 ) .ee 36
17 離場機器人 Remove The Robot From The Map 〈上游系統 -> Kuka 系統 ) pp 37
-- AMIR 地圖點位與區域相關接 口 eeesseeessecsneesneeesneesusesssessueesnscesueesnsessueesnessusesneeesusesneessneesneeeseeesnsensees 38
18 查詢所有 WCS 區域信息 Query All WCS Area Information 〔(上游系統 -> Kuka 系統 ) vee 38
19 區域內點位信息查詢 Query Nodes of the Area (上游系統 -> Kuka 系統) .es 39
20 查詢點位所屬區域 Query Area by Map Node 〈上游系統 -> Kuka 系統 ) .RN 41
21 查詢所有禁行區 Query All Forbidden Areas (上游系統 -> Kuka 系統 ) .pp 42
22 查詢指定禁行區 Query One Forbidden Area (上游系統 -> Kuka 系統 ) .Rs 44
23 更新指定禁行區的狀態 Update Forbidden Area Status (上游系統 -> Kuka 系統) .ee 45
24 查詢功能點位 Query Function Node (上游系統 -> Kuka 系統 ) eececccececssssscsesecsssecsescseseesevsvsveeees 47
-- 接口響應報文說明 ee eccccescsesssscscscscscscscscscscscscscscscscscscscscscscssvasssasasasssasssasacscscssssssssasscsescecscscseseeeseenenens 49
版本號          修改內容                         | 昌期 |修改人

2. 接口回調，除了 missionCode，和
missionStatus 都改成非必填

 2.0.0 1.輥簡車任務，新增輥簡取放完成後“| 20240708 李南

1.2.3               |  1.任務下發增加復合機器人類型 2023.11.22            陳文越

是否通知字段 actionlnform
2.料箱車任務，新增料箱取放完成後
是否通知字段 takeActionlnform

putActionlnform

取放交互一般用於與輸送線對接時使

 2.11.1   1. 新增接口， /jobQuery。 20240125 趙禾川

2. 更新 與禁區 文相關 API 以及 API
/missionCancel 的請求參數和請求體示

 2.11.2         ;   1. 新增接口: 20250214 張骞文

/queryRobByNodeUuidOrForeignCode.
2. 更新 API
/updateForbiddenAreaStatus 的請求參
BAS RAN

/queryContainerAll
(可以同時查詢入場和離場狀態的容

V1.0    05.2025    KUKA AMR                                                                                                            2/49

## Page 3

KUKA

/containerln

新增時可以指定容器校驗碼)

1. 新增接口: 20250414 AI, A
/robotMoveCarry

2. 新增接口:

/queryFunctionNode

3. 狀態回調新增 WAITFEEDBACK 狀態

類型〈流程節點等竺放行時回調該狀

態)

1. 新增接口: 20250414趙禾川
/insertRobot

2. 新增接口:

/removeRobot

3. 新增接口:

/chargeRobot

-- AMR 任務執行相關接口

1任務下發 Dispatch Mission (上游系統 -> Kuka 系統)

@ ApPl 基本信息
Ah | submitMisson
 PUES TR

APIURL http://[IP:Port]/interfaces/api/amr/submitMission

@ Headers

參數名稱          參數值              是否必須

公共請求參數說明

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

庫存組織 ID(或工
Ie 供應商代

(ea
T      任務類型:
he FE A)
動:RACK_MOVE
輥簡車搬運任務:
ROLLER_MOVE
料箱車搬運:
PICKER_MOVE
又車搬運:
FORKLIFT_MOVE
機器人移動:
MOVE
復合機器人搬運:
ROBOTICS_MOVE

PLE Ahh   eee
WHA: LIFT

輥簡類: ROLLER
料箱類:PICKER
車: FORKLIFT

加     Site   本

Integer

String
String
String

Boolean

String

String

V1.0    05.2025    KUKA AMR

L5G 級,1-99，
   小， 優先級

一
一
一
天需要流程 結束
後機器人保持人
鎖定狀態

當前小車的在
F務的鎖定狀

原則上不允許為
空，如果為空

就會通過通過機
器人具體信號或
容器類型編碼進
AT HET

支持指定多個，會
人人  個
適的執行任務;

支持指定多個，會
從多個中指定一個
合適的執行任務;
如果小車前一個任
務流程結束後執行
鎖車，該字段必填
批量任務下發時
有效

多用於流程結束
後，機器人放下
容器之後還需要
當前小車搬運的
場景

F             作業流程    完成後，

j於校驗解鎖是
AEWA

4/49

## Page 5

指定機器人停放區
域/點

oe                  —                                                      當前任務包含的流
程節點信息列表具
體字段參考一下章

如果是調用流程模
板，則次字段可以

1.1 貨架移動任務(missionType=RAck_MovE)

@ 請求參數說明
Integer                           序號 默認 1 為起始貨
架的起點，  , 後續需
要停留點的需要依次
遞增，終點值最大

                     E業路徑位置
FE業位置類型，
點位; NODE_POINT
區域: NODE_AREA

jPutpows | Boolean                              作業點位是否需要放
下貨架

we                   String                                                                     當前任務點結束後放
了策略:
iz: AUTO
2): MANUAL

waitingMillis          Integer                                   動觸發離開當
務節點的時間，默認 | passStrategy
單位:毫秒           是手動則可
不 填 ，
passStrategy
是自動則必
填

@ 請求報文示例
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
@ 請求返回報文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

}
1.2 4 fal 42 PIS ES (missiontype=ROLLER_MOVE)
@ 請求參數說明

sequence          Integer           T              序號 默認 1 AA
一個節點，後續需要

“position Sting    1

type                String              T                            on
點位;NODE
Xt: NODE

“bincode se SSCSC*SCR I
“rollerievel mteeer | FF OSS MAR | SRE

V1.0    05.2025    KUKA AMR

6/49

## Page 7

actionType                      String

actionConfirm

actionInform

@ 請求報文示例
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

在點位上需要執行的
動作

ROLLER_RECEIVE: 取
ROLLER_SEND: 方

當一 個點位
對應多個設
備時需要傳
入該值
當需要對接
輸送線且需
要自動上下
料時，需要
傳入對應的
執行指令;
如果是人工
上下料或者
是夾取料，
則可以不傳

Boolean   pf false      上下料前是否需要確
認

般用於與
輸送線對接
交互

Boolean   ae false      上下料後是否需要通
知

般用於與
輸送線對接
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
請求返回報文示例
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

T           搬運料箱序號，從 1
開始，也可以認為是
執行的優先級

TORR ee finite mcf
TR MRI Ae frteke nin
To | | 料箱所在的目標點位
了 |和料箱所在的目標村位

 false     取料箱時之前
要確認

 false     取料箱後是否需

F

—
—
—
—
ee |料箱號

           放料箱後是否需要通

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

@ 請求返回報文示例

{
"data": null,
"code": "0",
"message": null,
"success": true

}

1.4 X 4 RRIS LES (missiontype=FORKLIFT_MOVE)
@ 請求參數說明

sequence          Integer           T              序號 默認 1 為任務第
停留點的需要依次弟
增，終點值最大
|                      |

am Sting ST             fe

meee |       |
數           EYER
要確認

@ 請求報文示例
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

@ 請求返回報文示例

{
"data": null,
"code": "0",
"message": null,
"success": true

}
1.5  機器人移動 (missionType=MovE)

sequence          Integer                           序號 默認 1 為經過的
第一個點， , 後續需
要停留點的需要依次
遞增，終點值最大

-Sm | 上|一一  作業路徑位置

[si __J                                String                                                                 作業位置類型;
點位; NODE_POINT
區域:， NODE_AREA
passStrategy                 String                           F              AUTO          當前任務點結
行策略;
自動: AUTO
手動: MANUAL

waitingMillis                 Integer                         F                                   動觸發離開當前任 | 若
務節點的時間，默認 | passStrategy
單位:毫秒           是手動則可
不 填 ，
passStrategy
是自動則必
填

@ 請求報文示例
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
@ 請求返回報文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

}
1.6 復合機器人任務(missionrype=RoBoTics_MovE)
@     "yaa

er             Integer                                       序號 默認 1 為起始貨
架的起點，，後續需
要停留點的需要依次
遞增，終點值最大
| Cd
|

Position String          Ht {|  作業路徑位置

type                              String                                                                   作業位置類型;
點位， NODE_POINT
區域: NODE_AREA

“applicationName | Sting)  T | CLR
“parame | sting | SON

V1.0    05.2025    KUKA AMR                                                                                                           12/49

## Page 13

KUKA

pRB

—a——     a               AUTO   7 A EAE 結束後放
行策略;
Az: AUTO
手動: MANUAL

waitingMillis         Integer                                    動觸發離開當前任
務節點的時間，默認 | passStrategy
單位:毫秒          是手動則可
不 填 ，
passStrategy
是自動則必
填

@ 請求報文示例
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
@ 請求返回報文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

2 任務取消 Cancel Mission (上游系統 -> Kuka 系統)

@ ApPl 基本信息

 (ssi

APIURL http://[IP:Port]/interfaces/api/amr/missionCancel
  POST

@ Headers

aaa |

@ 請求參數說明

requests | Sting 3 TT) Rid, HEAP eae

加        =
—_         mm | ft   ~

pre frm | RRR |
了

cancelMode            String                      T            FORCE | 取消模式:                        當前

FORCE 強制取消，立即結       關亞
NORMAL 普通取消，不會 | 任
取消小車正在執行的任

務，等待小車把當前任務

執行完，再取消流程
REDIRECT_END 前往終點，      FORCE,
不會取消小車正在執行的 | 是具體還

V1.0    05.2025    KUKA AMR                                                                                                           14/49

## Page 15

任務，等待小車把當前任 | 需要看機
務執行完，如果目標點不 | 器人維度
需要降下貨架，則小車攜 | 是否支持
帶貨架前往終點，和否則空 | 取消

車移動至終點
REDIRECT_START 前往起
點，不會取消小車正在執
行的任務，等待小車把當
前任務執行完，如果目標
點不需要降下貨架，則小
車攜帶貨架回到起點，和否
則空車移動回到起點

reasm String | IF | |取消原因

@ 請求報文示例
{
"redquestld": "request202309250006",
"missionCode": "mission202309250004",

"containerCode": "",
"position": mu
"cancelMode": "FORCE",

"reason":
}
@ 請求返回報文示例
{
"data": null,
"code": "0",
"message": null,
"success": true

3 任務放行 Resume Workflow (上游系統 -> Kuka 系統)

@ API 基本信息
 節點任務完成後，上游對任務完成信息的反饋

APIURL http://[IP:Port]/interfaces/api/amr/operationFeedback

@ Headers

 application/ison |是 | |

@ 請求參數說明

 RIG HG wuidse fe)

V1.0    05.2025    KUKA AMR                                                                                                           15/49

## Page 16

‘missionCode [Sting | SSS*d 5
“containercode | Sting) SSS         eo
“position | Sting SSS        當前執行作業的節點

@ 請求報文示例

{
"requestid": "request202309250007",
"containerCode": "",
"missionCode": "mission202309250005",
"position": ""
}
@ 請求返回報文示例
{
"data": null,
"code": "0",

"message": null,
"success": true

}

4作業看板查詢接口 Query Jobs (上游系統 -> Kuka 系統)

@ ApPl 基本信息
API 名稱                 jobauerry eee
作業看板查詢接口

http://[IP:Port]/interfaces/api/amr/jobQuer
HTTP Method

@ Headers

@ 請求參數說明

流程實例 id
狀態

機器人編號
流程配置名稱

|
|
|
10: 待執行，20: 執行
中; 25: 等待放行;
28: 取消中; 30: 已完
成; 31: 已取消;35:
手動完成， 50: 告警;
60: 流程啟動異常
|
|

V1.0    05.2025    KUKA AMR                                                                                                           16/49

## Page 17

KUKA

“worlfiowCode | String «| [FF |  ，|尖本山       一 |
Smaps testngs | LPB         -
“ereateUsername | Sting FI

sourceValue      Integer                     來源
2: FOF AES; 3:
PDA 任務; 4: 設備任
3; 5: MLS 任務;6;
前端界面觸發，7: 流
程事件觸發;

Integer                       返回作業條數，默認返 |
回 10 條
@ 返回字段說明

 作業及中

eC ER
 suing HS
robots String ES

Integer    FE業狀態
0: 待執行， 20: 執行
H; 25: 等竺放行;
8: 取消中; 30: mo
成; 31: GRY:
FE 動完成; 50: ne
0: 流程啟動異常

workflowcode | String | WRB |
 AI
“maptode String SEEING
“targetceliCode stme Hine
“begincelicode | String im
“targetCeliCodeForeign | String) Hin RMA
“beginceliCodeForeign sting ASIA LIANG
   ee

warnFlag                     Integer        告警標志
0: 正常，1: 告警
Tie Se AS |
completeTime           String       流 fo    = 時 間 Cyyyy-
MM-dd HH:mm:ss )

spendTime                           Integer          TRE AE Be TA] CRD)            |
_createUsername | String |操作員 |

createTime               String       流程創建時間 (yyyy-
MM-dd HH:mmiss )

String    作業來源
INTERFACE: 接口平台;
PDA: PDA fh 發 ;
DEVICE: 設備觸發;
MLS: MLS 觸發，SELF，
前端界面觸發，EVENT:
流程事件觸發;

 物料信息              現場再

V1.0    05.2025    KUKA AMR                                                                                                           17/49

## Page 18

@ 請求報文示例〈僅供參考，根據實際需要填寫參數，所有字段非必填)

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

@ 請求返回報文示例
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

-- AMR 容器相關接口

5 容器入場 Insert Container (上游系統 -> Kuka 系統)

@ ApPl 基本信息
 containerin
 容器相關操作，入聲

APIURL http://[IP:Port]/interfaces/api/amr/containerin
  POST

@ Headers

[aa ae |

requestld          String   32   T       OR ids He 等
uuid32 位

containerType                 String                F                 容器類型
貨架:，RACK
料箱 :BIN(暫未實
見)
containerModelCode           String                                  容器模型編碼
chew-true
時必傳

容器

enterOrientation       ie s aT

@ 請求參數說明

                        cana
                         false           TSI Bd A a

a

a ae

-一二-一

a             String                                                            7 RI                      當

isNew=true
時可以指
定是否配
置校驗碼

withDefaultValidationCode    Boolean                   false | 配置容器默認校驗 | 4
ng                                      isNew=true

V1.0    05.2025    KUKA AMR                                                                                                           19/49

## Page 20

KUKA

@ 請求報文示例〈當前貨架入場)
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

@ 請求報文示例《貨架新增並入場,不配置校驗碼)

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

@ 請求報文示例《貨架新增並入場,配置指定校驗碼)

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

@ 請求返回報文示例

V1.0    05.2025    KUKA AMR

20/49

## Page 21

KUKA

"data": null,
"code": "oO",
"message": null,
"success": true

}

6 容器出場 Remove Container (上游系統 -> Kuka AA)

@ API 基本信息
 containerOut
 sec, a

APIURL http://[IP:Port]/interfaces/api/amr/containerOut

HTTP Method                POST

@ Headers

參數名稱         參數值             是否必用
Content-Type       application/json

@ 請求參數說明

@ 請求報文示例

{
"requestid": "request202309250009",
"containerType": "",
"containerCode": "10",
"position": "M001-A001-31",
"isDelete": false
}
@ 請求返回報文示例
{
"data": null,
"code": "0",
"message": null,
"success": true
}

V1.0    05.2025    KUKA AMR

5

貨架: RACK
BLA: BING AS
| 容器編號
| 容器出場位置
 Tea sa

21/49

## Page 22

KUKA

7 容器信息更新 Update Container (AN MH: 位置和空滿狀態的更
新) (上游系統 -> Kuka 系統)

@ ApPl 基本信息

 守信和更新

APIURL http://[IP:Port]/interfaces/api/amr/updateContainer
  POST

@ Headers

@ 請求參數說明

貨架: BUCKET
料箱:BIN(暫不支持)

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
@ 請求返回報文示例
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

8接口回調 Mission Status Callback (Kuka 系統 -> 上游系統)

@ ApPl 基本信息

 黨間作

APIURL http://[IP:Port]/interfaces/api/amr/missionStateCallback
  POST

@ 請求參數說明

mmcde Sting 3 T yd |
‘viewBoardType se | FF | 人sk
“containercode sme FF Aha
merge [sting [fF a sn

                       Br FEA

message
ea |

@ 請求報文示例
{
"missionCode": "mission202309250005",

IF | |當
a ee
String                                T                                 作業當前狀態

開始移動: MOVE_BEGIN
到達任務節點;ARRIVED
i 升 完 成 :
UP_CONTAINER
放 下完成 :
DOWN_CONTAINER
輥 ff 上 料    :
ROLLER_RECEIVE
 fl 下 -    :
ROLLER_SE
料 8 取 料    :
PICKER_RECEIVE
料箱下料完成:
PICKER_SEND
又車又取完成
FORK_UP
叉車放下完成;
FORK_DOWN
等竺放行， WAITFEEDBACK
任務完成: COMPLETED
任務取消完成: CANCELED

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

9 容器模型查詢接口 Query Container Model Codes (上游系統 -> Kuka 系
統)

@ ApPl 基本信息

 查詢所有已配置容器模型

APIURL http://[IP:Port]/interfaces/api/amr/queryAllContainerModelCode

@ 無請求參數
@ 請求返回報文示例
{
"code": "0",
"message": null,
"success": true,
"data": [
"10001"
]
}

10 查詢容器模型推薦的可存放區域 Query Area Codes By Conatiner
Model (上游系統 -> Kuka 系統)

@ ApPl 基本信息

 queryAreaCodeForContainerModel
 查詢容器模型推薦的可存放區域

APIURL http://[IP:Port]/interfaces/api/amr/queryAreaCodeForContainerModel
 GET

@ 請求參數說明
| 容器模型編碼              |

containerModelCode

Te
noContainerFirst      Boolean             F        false | 是否區域內無容器優先，
ture 是
false &

@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/queryAreaCodeForContainerModel?containerModelCode=10001&
noContainerFirst=false

V1.0    05.2025    KUKA AMR                                                                                                           24/49

## Page 25

KUKA

@ 請求返回報文示例
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

11-1 容器信息查詢接口(僅查詢入場狀態容器) Query Containers (上游
系統 -> Kuka 系統)

@ API 基本信息
 containerQuery
 容器信息查詢接口(僅查詢入場狀態的容器)

APIURL http://[IP:Port]/interfaces/api/amr/containerQuery
HTTP Method

@ Headers

參數名稱          參數值              是否必須

© 請求參數說明

gen

nodeCode
 string

|
|
|
|
emptyFullStatus            pm]

@ 返回字段說明

containerCode   容器編碼

點位編碼
容器模型編碼

—
—
|
—
  容器的空滿狀態;

 容器編四
| |
| |

20
滿1
全部2

nodeCode  點位編碼                   |
Integer         入場/離場狀態
0-離場，1-入場

containerModelCode 容器模型

V1.0    05.2025    KUKA AMR                                                                                                           25/49

## Page 26

KUKA

emptyFullStatus
isCarry                      Integer       搬運狀態 0-靜止 ，1-搬
運中
String           a 半角度
String | 容器校驗碼 |
mapCode                                            地圖 人
| districtCode          | String     | 地圖片區編碼            |

orientation
containerCheckCode

@ 請求報文示例〈查詢指定容器)
{
"nodeCode": "M001-A001-30",
"containerModelCode": "10001",
"containerCode": "10",
"areaCode": "A000000014",
eee 2

e aR 回報文示例

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

11-2 容器信息查詢接口(同時查詢入場和離場狀態容器) Query
Containers (上游系統 -> Kuka 系統)

@ API 基本信息
API 描述

[APIURL i

全一

@ Headers

containerQueryAll
容器信息查詢接口(同時查詢入場和離場狀態的容器)

http://[IP:Port]/interfaces/api/amr/containerQueryAll                      |
POST

V1.0    05.2025    KUKA AMR                                                                                                           26/49

## Page 27

參數名稱 — | Set     是否必須
 spplicatonfson | 是 | |

_

@ 請求參數說明

containerCode
 String
 String

emptyFullStatus                                                  容器的空滿狀態:
20
滿1
                               容器入場離場狀態
離場0
入場1

Integer         入場/離場狀態
0-離場，1-入場
_containerModelCode _

@ 返回字段說明

containerModelCode | String           容器模型

emptyFullStatus   0-空 1-滿

 這 me me
運中

 EAI

“containercheckCode | sting | HME ，
mapcode | Sting | SMS
“distritCode | Sting | SIRS

@ 請求報文示例〈查詢指定地圖所有容器)

{
"mapCode": "ABC"
}
@ 請求返回報文示例
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

-- 機器人相關接口

12 根據點位 UUID 或外部編碼查詢機器人 Query robot by node UUID or
node foreign code (上游系統 -> Kuka 系統)

@ ApPl 基本信息

API 名稱                   queryRobByNodeUuidOrForeignCode

API 描述                   根據點位 UUID 或點位外部編碼查詢機器人

APIURL http://[IP:Port]/interfaces/api/amr/queryRobByNodeUuidOrForeignCode

@ Headers

是        和

Content-Type          application/json

@ 請求參數說明

V1.0    05.2025    KUKA AMR                                                                                                           28/49

## Page 29

KUKA

PoP yy

@ 返回字段說明

點位 UUID 或點位外部編
碼

  機器人編碼               |
  機器人型號               |

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

 地圖編碼
文鄉

  工廠或者倉庫編碼
  機器人持有容器的編碼

 1-離 場 ，2-離 線 ，3-空
As 4-任務中;， 5-充電
中;，6-更新中，7-噶常

 是否占用: OA; 1-
占用

  量〈單位，百分比)

String     [器人當前所處坐標
   單位: 毫米)

    L       當前所處坐標
y(4     毫米)

 機器人項升狀態 CA: 項
Ft; O: 未頂升)

Integer     定位置信度: 0-不可信;

 運行時長

  件

 片區編碼

wn
o
a
5
oa

String | 右電機溫度

String | 機器人所在節點的外部
編碼

wn
o
a
5
oa

任務號

EEE
當使用節
點的外部
編碼查詢
機器人
時，該字
BORA
值。

29/49

## Page 30

KUKA

@ 請求報文示例
1) 示例1
{
"nodeCode": "startNode1, startNode2"
}

2) 示例 2
{

"nodeCode": "test01-5-91"
}

@ 返回報文示例
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

13 機器人信息查詢接口 Query Robots (上游系統 -> Kuka 系統)

@ API 基本信息
API和稱 robotauevy
 機器人信息查詢接口(蒜認查所有)

APIURL http://[IP:Port]/interfaces/api/amr/robotQuery

@ Headers

參數名稱   參數值    是否必須

robotd lstrnge | IF | |機器人編碼           1) 如果參
            號              六
      地圖編碼〈禁有    | 空，則默認

F

@ 請求參數說明

mapCode                          String
果為空)        礙全部。

|
-                                    Te        7
floorNumber       String                        og ( 48 F        2) 請求參
數 mapCode
和
floorNumber
必須一起傳
ih.

@ 返回字段說明

  機器人綱碼

 機器人型號
 SFE

V1.0    05.2025    KUKA AMR                                                                                                           32/49

## Page 33

KUKA

floorNumber                        String           片區編碼

buildingCode                 String        工廣或者倉庫編碼

containerCode                    String          機器人持有容器的編碼

status                         Integer       1-離 場 ，2-離 線 ; 3-空
PAs 4-任務中;， 5-充電
中;，6-更新中，7-異常

occupyStatus                 Integer       是否占用: 0-未占用; 1

上

batteryLevel                   Double       Hie (AAT: 百分比)

nodeCode                            String            當前點位

X                     String      機器人當前所處坐標
x(單位: 毫米)

Y                     String      機器人當前所處坐標
y(單位:  毫米)

robotOrientation                  String           機器人當前角度

missionCode                        String            當前任務號

liftStatus                      Integer       機器人項升狀態 C1: Tit
升; 0: 未頂升)

reliability                  Integer      定位置信度: 0-不可信，
1-可信

runTime                      String        運行時長

karOsVersion                String        軟件版本

mileage                         String         運行總裡程

leftMotorTemperature       String        左電機溫度

rightMotorTemperature     String        右電機溫度

liftMotorTemperature          String           頂升電機溫度

rotateMotorTemperature | String           ees WL

rotateTimes                  Integer       托盤旋轉次數

liftTimes                      Integer       托盤頂升次數

@ 請求報文示例〈當前貨架入場)

V1.0

{

"floorNumber": "A001",
"mapCode": "M001",
"robotld": "13",
"robotType": "KMP600I'

}

請求返回報文示例

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

上游下發
任務號

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

14 下發機器人移動搬運任務 Dispatch Move Carry Task (上游系統 ->
Kuka 系統)

@ ApPl 基本信息

robotMoveCarr
下發機器人移動搬運任務，只在任務列表展示，作業看板不可見。

Dispatch a move carry task to the robot. The function is the same as

which in the Map Monitor, directly dispatched to the Mission Manager,
only displayed in the Mission List, and not visible on the Job Board.
http://[IP:Port]/interfaces/api/amr/robotMoveCarry

HTTP Method

@ Headers

 applicationfison |是 | |

a

@ 請求參數說明

| 機器人編號

目標點位編碼〈點位
UUID )

a ae
_containerCode | String | TO 容器編號
|

targetNodeCode

V1.0    05.2025    KUKA AMR                                                                                                           34/49

## Page 35

missionCode                                任務號，下發給 mission
manager的任務號。不傳
會自動生成。

@ 返回字段說明
下發成功時，返回任務號

@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/robotMoveCarr
{
"robotld": "10001",
"containerCode": "C0002",
"targetNodeCode": "TEST-1-45"

}
@ 返回報文示例
示例1
{
"data": "I-CARRY-1744609148201",
"code": "0",

"message": "OK",
"success": true

}

15 機器人充電 Dispatch Charge Robot Task (上游系統 -> Kuka 系統)

@ ApPl 基本信息

API名稱 | chargeRobot
機器人充電，只在任務列表展示，作業看板不可見。

Dispatch charge task to the robot. The function is the same as which in

the Map Monitor, directly dispatched to the Mission Manager, only
displayed in the Mission List, and not visible on the Job Board.

http://[IP:Port]/interfaces/api/amr/chargeRobot
POT

HTTP Method                POST

@ Headers

 applicationfison |是 | |

@ 請求參數說明

| 機器人編號

Te                                          |
necessary                       Integer                          T                             是否強制生成充電任務
1-一定生成任務並排

V1.0    05.2025    KUKA AMR                                                                                                           35/49

## Page 36

BA; 0-低 於 lowestLevel
則去充電，   無資源的情

可 直接完成

“targetlevel finger] STS,

lowestLevel                 Integer                                                WEF 電的電量，單位
=F Coe
必傳
== ED
wes 自動生成

@ 返回字段說明

無

@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/robotMoveCarr
{

"lowestLevel": 5,
"necessary": O,
"robotid": "2",
"targetLevel": 90

}
@ 返回報文示例
{

"data": null,
"code": "oO",
"message": "OK",
"success": true

16 入場機器人 Insert The Robot Into The Map (上游系統 -> Kuka 系統)

@ API 基本信息

 ingertRobot
入場機器人，該接口目前只文持真車。
Insert the robot into the map. Only support real AMR

currently (distinguished from simulated AMR).

APIURL http://[IP:Port]/interfaces/api/amr/insertRobot
  POST

@ Headers

Content-Type       application/json   SS

@ 請求參數說明

V1.0    05.2025    KUKA AMR                                                                                                           36/49

## Page 37

robotd | String TT 機器人編號               |
jcelcode __} String | fr} BBA AM 位       |

@ 返回字段說明

無

@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/insertRobot
{

"cellCode": "TEST-1-69",
"robotid": "2",
"synchroContainer": 1

}
@ 返回報文示例
{

"data": null,
"code": "oO",
"message": "OK",
"success": true

}

17 離場機器人 Remove The Robot From The Map (上游系統 -> Kuka 系
統)

@ API 基本信息

API 描述      離場機器人
Remove the robot from the map

APIURL http://[IP:Port]/interfaces/api/amr/removeRobot

@ Headers

Temes [acco |

fn

@ 請求參數說明

ee |sme | ft}  機器人編號

@ 返回字段說明

V1.0    05.2025    KUKA AMR                                                                                                           37/49

## Page 38

KUKA

無
@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/removeRobot
{
"robotld": "2",

"withContainer": 1

}
@ 返回報文示例
{
"data": null,
"code": "0",
"message": "OK",
"success": true

}

-- AMR 地圖點位與區域相關接口

18 查詢所有 Wocs 區域信息 Query All WCS Area Information (上游系統
-> Kuka 系統)

@ ApPl 基本信息

  查詢所有 WCS 區域信息 Query All WCS Area Information

APIURL http://[IP:Port]/interfaces/api/amr/areaQuery

@ Headers

aaa | 一

 nL
 pc

|
|
areaType                Integer       區 TE 區  3c fF
5監管區 7又車搬運

V1.0    05.2025    KUKA AMR                                                                                                           38/49

## Page 39

KUKA

@ 請求報文示例
@ 請求返回報文示例

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

19 區域內點位信息查詢 Query Nodes of the Area (上游系統 -> Kuka K

統)

@ ApPl 基本信息

api 名稱               | areaNodesQuery
API 描述                區域內點位 (外部編碼) 信息查詢 Query Nodes of the Area

V1.0    05.2025    KUKA AMR

39/49

## Page 40

KUKA

APIURL http://[IP:Port]/interfaces/api/amr/areaNodesQuery

HTTP Method                POST

@ Headers

Content-Type       application/json   SS

@ 請求參數說明

areacodes | listcSting> | TT) LRU       _—
@ 返回字段說明

 ra        Po

List<String> | 點位外部編碼的集合。
若該區域內所有點位均
無外部編碼，則該字段
可能為空

@ 請求報文示例

1) 示例1
{
"areaCodes": ["A000000013","A000000014"]
}
2) 示例2
{
"areaCodes": ["1-song-1732612877805"]
}

@ 返回報文示例
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

3) 示例 2，區域內點位均無外部編碼
{
"data": [
{
"areaCode": "1-song-1732612877805",
"nodeList”: [

1],

"code":  "oO",
"message": null,
"success": true

20 查詢點位所屬區域 Query Area by Map Node (上游系統 -> Kuka 系
統)

@ ApPl 基本信息

  查詢點位所屬區域 Query Area by Map Node

APIURL http://[IP:Port]/interfaces/api/amr/queryWCSAreaByMapNode

@ Headers

| content-type | application/json |是 | | Cd

nodeuud | String, | IT | |點位uup
@ 返回字段說明

@ 請求參數說明

areaCode                          String             區域編碼
area code

V1.0    05.2025    KUKA AMR                                                                                                           41/49

## Page 42

區域可放置的容器的模 | 不一定有
型編碼

@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/queryWCSAreaByMapNode?nodeUuid= 1-song-44

@ 返回報文示例
1) 查詢到點位所屬區域，區域未配置可放置的容器模型編碼
{

"data": {
"areaCode": "1-song-1732612831825",
"containerModelCode": ""

1,

"code": "0",

"message": null,

"success": true

}
2) 查詢到點位所屬區域，區域未配置可放置的容器模型編碼
{

"data": {

"areaCode": "1-song-1732612877805",
"containerModelCode": "600i"
1,
"code": "0",
"message": null,
"success": true

}
3) 未查詢到點位所屬區域
{
"data": null,

"code": "100001",
"message": "No such node in the graph.[7788]",
"success': false

21 查詢所有禁行區 Query All Forbidden Areas (上游系統 -> Kuka 系統)

@ API 基本信息
API 名稱                   queryAllForbiddenAreas
API 描述                     查詢所有禁行區 Query All Forbidden Areas

API URL                    http://[IP:Port]/interfaces/api/amr/queryAllForbiddenAreas
| HTTP Method      | GET

V1.0    05.2025    KUKA AMR                                                                                                           42/49

## Page 43

KUKA

@ Headers

Content-type | applicationvison |是 | |

@ 請求參數說明
無

@ 返回字段說明
forbiddenAread integer | 全id
 SF Oa        _

floorNumber
  禁行區描述

forbiddenAreaStrategy | Integer          禁行策略:0-默認;1-人允許
有車
Integer        狀態:0-禁用;1-啟 A;
效中;3-失效中
enableType                 Integer        E效方式:0-立即生效;
按給定時間段生效

@ 請求報文示例
無

@ 請求返回報文示例
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

22 查詢指定禁行區 Query One Forbidden Area (上游系統 -> Kuka K
統)

@ API 基本信息
  queryOneForbiddenArea

  查詢指定禁行區 Query One Forbidden Area

PPorUntracesyapyamyatenOnetorhdeneea |  http://[IP:Port]/interfaces/api/amr/queryOneForbiddenArea

HTTP Method                GET

@ Headers

forbiddenAreald          Integer                                            PEAT IX id
forbidden area ID
@ 返回字段說明

字段名稱  字段描述                        項目用途
型

forbiddenAreald         Integer MTR ig
 地圖片區編碼

@ 請求參數說明 _

floorNumber        sme RE OSCC‘“‘~*~“—s*~*~S~S~S
 27 DCT          an

forbiddenAreaStrategy   禁行策略:0-默認;1-人允許
有車

V1.0    05.2025    KUKA AMR                                                                                                           44/49

## Page 45

Integer     狀態:0-禁用;1-啟 A;
效中;3-失效中
enableType             Integer     生效方式:0-立即生效;
按給定時間段生效

@ 請求 URL 示例
http://[IP:Port]/interfaces/api/amr/queryOneForbiddenArea?forbiddenAreald=2

@ 返回報文示例

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

23 更新指定禁行區的狀態 Update Forbidden Area Status (上游系統 ->
Kuka 系統)

@  Pl 基本信息

 updateForbiddenAreastatus
  更新指定禁行區的狀態 Update Forbidden Area Status

APIURL http://[IP:Port]/interfaces/api/amr/updateForbiddenAreaStatus
HTTP Method                POST

@ Headers

@ 請求參數說明

forbiddenAreald          a et a                                                                              在請求體中，

forbiddenAreald 或
forbiddenAreaCode
只能選擇其中一

V1.0    05.2025    KUKA AMR                                                                                                           45/49

## Page 46

個作為參數。

status                                String

@ 返回字段說明
無
@ 請求報文示例 request body
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

@ 返回報文示例 response body
1) 狀態修改成功後返回的報文
{
"data": null,
"code": "0",
"message": null,
"success": true
}
2) 狀態修改失敗後返回的報文
錯誤原因: 該禁行區的狀態已經為0〈禁用) ，仍舊修改其狀態為0，則會收到如下錯誤。如果狀態已
經為1〈啟用) ，再次修改其狀態為1，也會收到類似錯誤。
{
"data": null,
"code": "100001",
"message": "RCS returns error[[D]com.kuka.rcs.bd.common.grpc.util.CommonGrpcErrorCode.16[can not
update [ MAP_ZONE_STATUS_DISABLE ]]]",
"success": false
}
3) 傳參錯誤
錯誤原因: 每次只能選擇 forbiddenAreald 和 forbiddenAreaCode 中的一個作為請求參數。
{

"data": null,
"code": "100001",

"message": "only one of area ID or area code can be filled in",
"success": false

V1.0    05.2025    KUKA AMR                                                                                                           46/49

## Page 47

KUKA

24 查詢功能點位 Query Function Node (上游系統 -> Kuka 系統)

@ APL 基本信息
   queryFunctionNode
  查詢功能點位 Query Function Node

APIURL http://[IP:Port]/interfaces/api/amr/queryFunctionNode
  POST

@ Headers

| content-type | application/json |是 | | Cd

© 請求參數說明

functionType                   T                              功能點位類型

                    nae
                    ae

V1.0    05.2025    KUKA AMR                                                                                                           47/49

## Page 48

robotTypeClass | String | FO | |機器人類型編碼
containerModel           Strng | IF |容器模型編碼

@ 返回字段說明

| nodeCode         | String
| externalCode

|  String
functionType                  Integer

| 點位編碼 CUUID)    |
| 點位外部編碼     |     |

功能點位類型

containerModelCode

Bh DL SC HF BA A eh eA Sh
碼

robotTypeCode                  List<String> | 點位支持的機器人類型
|                           編碼

@ 請求報文示例
{

List<String>

"floorNumber": "1",
"functionType": 1,
"mapCode": "TEST"

@ 返回報文示例
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

-- 接口響應報文說明

@ 成功響應
{

"code": "oO",
"message": mu
"success": true,

"data": wit

}
@ FMA py
{
"code": mn //異常代碼
"message": " //Re it fa
"success": false,
"data": null

}
異常信息說明:

An
was
PRIA
eet

cer
BARRIS
酸硯是

V1.0    05.2025    KUKA AMR

49/49