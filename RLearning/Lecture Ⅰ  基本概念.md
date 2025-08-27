# Lecture Ⅰ 基本概念



#### 所有概念均采用 grid-world example讲解

## 目的：找到一个比较好的“路径”到达目标

## state：

agent相对于环境的一个状态

## state space：

将所有state放在一起的集合

action，

## action space：

依赖state，不同状态对应的action space不一样

## policy：

在当前的state，应该去采取哪个action，策略分确定性的和不确定性

## reward：human-machine-interface（r）

一个标量，agent采取一定行为后得到的值，一般来说正数代表鼓励该行为，负数代表不鼓励（数值相反也可以）

也可以用 tabular representation表示reward（表格形式，和policy的表格一样）。但只能表示deterministic的情况（即采取某一个action后的reward是定值，实际上-stochastic-得到的reward是不确定的）

## trajectory：state-action-reward的逻辑链

即从某个起点出发的一条路径

## return：回报

针对一个trajectory而言的，沿着一条trajectory把所有的reward都加起来的综合

return = rs1 + rs2 + rs3 + ......

当没有终止条件时，到达目标点后return会持续增加直到发散。所以引用dicount rate（γ）

## discount rate：折扣率γ∈（0，1）

discount return = γ·rs1 + γ^2·rs2 + γ^3`rs3 + ......

作用：平衡未来action和当前action的reward，折扣率越小越近视

## episode task（trial）：一个有限步的trajectory

与之对应的就是continuing task

## terminal state：终止状态

一个episode通常都会伴随terminal state

## continuing state：持续状态（理论上不会终止）

## 课程里会将terminal state转换成continuing state

以便将continuing task和episode task统一用一种数学方法解决。具体来说会把target state当成普通的一个状态，agent会为了得到更多的reward一直在target state原地踏步。

## Markov decision process（MDP）：马尔科夫决策过程

一个MDP包含很多要素

### 要素1：set集合

State状态 S

Action行动 A

Reward回报 R

### 要素2：probability distribution

:在状态s采取行动a，跳到状态s‘的概率

Reward probability：在状态s采取行动a，得到回报r的概率

### 要素3：Policy

在状态s的一个策略，会显示在当前状态采取action a的概率

### Markov property： 又名memory less  property

和历史无关的性质

即State transition probability 和 Reward probability在当前状态的概率只与当前采取的action有关，与之前所有状态的action无关。

#### 当Markov decision process的decision（policy）确定了，Markov decision process就变成了Markov process



| 名称 |                    对应解释                     |
| :--: | :---------------------------------------------: |
| v_t  |       状态s的期望价值，v_t也就是v_pi（s）       |
| G_t  |      一个trajectory所对应的discount return      |
| r_i  | 对应状态的单步即时reward ，是构成v和G的基本元素 |
|  R   |           一个trajectory多步的return            |

#### 注意区分单步和多步，discount