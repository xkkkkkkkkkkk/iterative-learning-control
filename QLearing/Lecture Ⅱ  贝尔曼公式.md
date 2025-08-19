# Lecture Ⅱ   贝尔曼公式

#### 核心概念： state value

#### 重要工具：Bellman equation

## 1. Motivating examples

return 很重要，可以用来评估一个策略

### 计算return的方法（此处将return写成  *v*  ）

### method 1：定义（此处都是continuing state 无限循环）

![image-20250819134157314](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250819134157314.png)
$$
v_i表示从s_i出发的return\\
v_1 = r_1 + γ·r_2 + γ^2·r_3\ldots\\
v_2 = r_2 + γ·r_3 + γ^2·r_4\ldots\\
v_3 = r_3 + γ·r_4 + γ^2·r_1\ldots\\
以此类推
$$

#### method 2： Bootstrapping（自举：从自身出发开始迭代）

$$
v_1 = r_1 + (γ·r_2 + γ^2·r_3\ldots) = r_1 + γ·v_2\\
v_2 = r_2 + (γ·r_3 + γ^2·r_4\ldots) = r_2 + γ·v_3\\
v_3 = r_3 + (γ·r_4 + γ^2·r_1\ldots) = r_3 + γ·v_4\\
v_4 = r_4 + (γ·r_1 + γ^2·r_2\ldots) = r_4 + γ·v_1\\
$$

从不同状态出发得到的return依赖于从其他状态出发得到的return

将上述式子转换成矩阵形式
$$
\underbrace{ \begin{bmatrix}v_1 \\ v_2 \\ v_3 \\ v_4 \end{bmatrix}}_{v}= \begin{bmatrix}r_1 \\ r_2 \\ r_3 \\ r_4 \end{bmatrix} + \begin{bmatrix}\gamma·v_2 \\ \gamma·v_3 \\ \gamma·v_4 \\ \gamma·v_1 \end{bmatrix} = 
\underbrace {\begin{bmatrix}r_1 \\ r_2 \\ r_3 \\ r_4 \end{bmatrix} }_{r}+ \underbrace{ \begin{bmatrix}0&1&0&0 \\ 0&0&1&0 \\ 0&0&0&1 \\ 1&0&0&0 \end{bmatrix} }_{P} \underbrace{\begin{bmatrix}v_1 \\ v_2 \\ v_3 \\ v_4 \end{bmatrix}}_{v}\\
v = r + \gamma Pv
$$

$$
求解return矩阵v
\\v = r·(I-\gamma P)^{-1}r
$$

#### 上式 v 即为贝尔曼公式（for the deterministic problem）

## 2. State value ----定义为 $v_\pi(s)$

考虑一个单步过程
$$
S_t \xrightarrow{\text{A_t}} R_{t+1},S_{t+1}
$$
t,t+1：离散时间

$S_t$ t 时刻状态 s

$ A_t $ : 状态$S_t$ 采取的行动$ A_t$

$R_{t+1}$ : 行动$A_t$ 后得到的reward

$S_{t+1}$  ：行动$A_t$ 转移到的状态

每个变量都有对应的概率分布，其在各个时刻各个状态的值都有对应的概率分布所决定

上述单步过程推广至多步过程
$$
S_t \xrightarrow{{A_t}} R_{t+1},S_{t+1} \xrightarrow{{A_{t+1}}} R_{t+2},S_{t+2} \xrightarrow{{A_{t+2}}} R_{t+3},S_{t+3} \ldots
$$
对应的discounted return为
$$
G_t = R_{t+1} + \gamma R_{t+2} + \gamma ^2R_{t+3}\dots
$$
$G_t$ 也是一个随机变量因为对应的$R_t$ 等就是随机变量

#### $G_t$ 是一个trajectory对应的discounted return，State value就是$G_t$ 的期望值。

state value 是一个条件期望，表达式如下，其中状态要为具体值
$$
v_\pi(s) = E[G_t|S_t = s]
$$
关于$v_\pi(s)$说明

1.$v_\pi(s)$是关于s的一个函数，s不同得到的trajectory不同，得到的discounted return也不同，$v_\pi(s)$也就不同

2.$v_\pi(s)$是基于策略$\pi$的

3.state value的value也代表对应状态的value，也代表对应策略的value。

![image-20250819151002849](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250819151002849.png

## 3.  Bellman equation： Derivation推导

考虑一个trajectory     $S_t \xrightarrow{{A_t}} R_{t+1},S_{t+1} \xrightarrow{{A_{t+1}}} R_{t+2},S_{t+2} \xrightarrow{{A_{t+2}}} R_{t+3},S_{t+3} \ldots$ 

对应的return 就是 $ G_t = R_{t+1} + \gamma R_{t+2} + \gamma^2R_{t+3} \\ \quad =R_{t+1} + \gamma (R_{t+2}+ \gamma R_{t+3})\\ \quad=R_{t+1}+\gamma G_{t+1}$ ,即由立即得到的immediate reward 加上下一个时刻能得到的discount return

将这两部分分为immediate reward 和future  discounted return ，$G_t$ 对应的state value也就可以分为两部分（期望相加性质）
$$
v_\pi(s) = E[G_t|S_t = s]\\ \qquad \ =E[R_{t+1} + \gamma G_{t+1}|S_t = s]\\\qquad\ =E[R_{t+1}|S_t= s]+ \gamma E[G_{t+1}|S_t= s]
$$
分别分析公式两部分的形式，得到贝尔曼公式

#### ①immediate reward：（一个状态下，采取不同的action有对应的概率，对应的action能得到的不同reward也有对应的概率）

$$
E[R_{t+1}|S_t = s] = \sum_{a}{\pi(a|s)E[R_{t+1}|S_t = s,A_t = a]}\\\qquad\qquad\ = \sum_{a}{\underbrace{ \pi(a|s)}_{采取action对应的概率}}\overbrace{ \sum_{r}p(s|s,a)}^{所有reward对应的概率和}r
$$

####  

#### ②future discounted return： 类似①，只是r变为了$ v_\pi(s)$

$$
E[G_{t+1}|S_t = s] = \sum_{s'}{E[G_{t+1}|S_{t+1} = s']p(s'|s)}\\\quad =\sum_{s'}{p(s'|s)\ v_\pi(s')}\\\qquad\qquad\qquad\  = \sum_{s'}{v_\pi(s')}\sum_{a}{\pi(a|s)\ p(s'|s,a)\ }
$$

#### 因此，对应的$v_\pi(s)$ 合并后的贝尔曼公式如下

$$
v_\pi(s) = \sum_{a}{\pi(a|s)}\Big[\sum_{r}{p(r|s,a)r + \gamma \sum_{s'}{p(s'|s,a)v_\pi(s')}} \Big]
$$

需要说明的是，贝尔曼公式描述的是state-value的关系，该式子包括两项：当前项和未来项

贝尔曼公式对状态空间中的所有状态都成立，如果有n个状态那么就会有n个对应的贝尔曼公式

#### 贝尔曼公式也是为了计算state value的，计算方法还是Bootstrapping，将n个状态的贝尔曼公式联立。

$\pi(a|s)$ 是依赖于policy的，所以求解贝尔曼公式的过程也可以叫做 policy evaluation（策略评估）

$p(r | s,a)$ 和 $p(s' | s,a)$ 代表的是dynamic model动态模型，当没有模型时也可以求解贝尔曼公式。

## 4.  Bellman equation： Matrix vector form

## 5.  Bellman equation： Solve the state value

## 6. Action value

## 7. Summary

