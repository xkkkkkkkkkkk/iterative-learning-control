# Lecture  Ⅷ  Value Function Approximation

## 1. Motivating  example —— curve fitting

离散状态下的state 和 action都默认存储在表格中，这样做的优点是直观且易于分析；但当面临较大的或连续的state/action  space，表格形式就很难处理了。
缺点：1.存储  2.泛化能力

#### 当有很多采样点state value时，考虑用函数曲线来拟合

一、首先考虑一个最简单的直线来拟合点，假设直线等式为
$$
\hat{v}(s,w) = as + b  = \underbrace{[s,1]}_{\phi^T(s)}\underbrace{\begin{bmatrix}a \\ b\end{bmatrix}}_{w} = \phi^T(s)w\\
其中，w是参数向量；\phi(s)是特征向量，\hat{v}(s,w)对于w是线性的
$$
 值函数近似的优点：
①用表格表示需要存储非常多的state value，但现在只需要两个参数a和b
②每次需要使用变量s的值，只需要计算函数$\phi^T(s)w$  
==与此同时也有缺点== ，函数逼近的值不能够准确的表示 state value  

二、考虑用二次曲线进行拟合
$$
\hat{v}(s,w) = as^2 + bs + c  = \underbrace{[s^2,s,1]}_{\phi^T(s)}\underbrace{\begin{bmatrix}a \\ b\\c\end{bmatrix}}_{w} = \phi^T(s)w\\
$$
这种情况下，w和$\phi(s)$ 的维度都增加了，但对应的拟合值能够更精确
并且即便$\hat{v}(s,w)$ 对于s是非线性的，但是对于w还是线性的。这种非线性的性质仅存在于$\phi(s)$上。

### 总结：

Idea：使用参数化的函数$\hat{v}(s,w) \approx v_\pi(s)$来近似state value和action value,其中w 是参数向量。
Advantage：存储优势：函数中的参数向量w的维度要远低于需要存储的|S|的维度
					 泛化能力：当状态s被访问到时，会使参数w开始更新，从而使得一些没有被访问到的状态									   也随着被更新，通过这种方式，逼近的value能够适用于一些未曾访问到的状态

![image-20250829140334902](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250829140334902.png)

## 2. Algorithm for state value estimation

### objective function

目的是优化参数w使得$\hat{v}(s,w)$ 对于每一个状态s都能最优逼近$v_\pi(s)$ 这是一个policy evaluation的问题。

找到最优的w，需要两个步骤
一、定义目标函数
二、推导出能够优化目标函数的算法

### 目标函数形式

$$
J(w) = E[(v_\pi（s) - \hat{v}(S,w) )^2]
$$

goal:找到最优的w使得目标函数J（w）最小，S作为一个随机变量，在求期望是需要确定其概率分布
有两种方法确定其概率分布

一、采用平均分布
把所有状态s都看作出现概率相等的， 若|S|为所有状态数量的话，此时目标函数为
$$
J(w) = \frac{1}{|S|}\sum_{s\in S}(v_\pi（s) - \hat{v}(S,w) )^2
$$
实际上所有状态并不一定同等重要，这种方法没有考虑到给定策略下马尔可夫过程的实际动态情况。

二、稳态分布
描述了马尔可夫过程中的long -run behavior (即策略于环境==长期交互==后，存在长期访问某一状态的行为)

#### 令$d_\pi(s)_{s \in S}$表示在策略$\pi$ 下马尔可夫过程的稳态概率分布，则对应的目标函数可以表示为

$$
J(w) = E[(v_\pi（s) - \hat{v}(S,w) )^2] = \sum_{s\in S} d_\pi(s)(v_\pi（s) - \hat{v}(S,w) )^2
$$

由于经常被访问的状态有着更高的$d_\pi(s)$ ,所以对应的他们在目标函数中的权重也会更高。

稳态分布也可以不经过长期交互直接求出：$d_\pi^T = d_\pi^TP_\pi$ ,其中$P_\pi$为状态转移矩阵（描述从状态s转移到状态s‘的概率分布）

### optimization algorithm

#### 为了最小化目标函数，使用梯度下降算法GD：$ w_{k+1} = w_k = \alpha_k \nabla_wJ(w_k)$ ,其中true gradient为

$$
\begin{split}
\nabla_wJ(w_k) &=\nabla_wE[v_\pi(S) - \hat{v}(S,w)^2]\\
&=E[\nabla_w(v_\pi(S) - \hat{v}(S,w))^2]\\
&=2E[(v_\pi(S) - \hat{v}(S,w))(-\nabla_w\hat{v}(S,w))]\\
&=-2E[(v_\pi(S) - \hat{v}(S,w))\nabla_w\hat{v}(S,w)]
\end{split}
$$

true gradient 计算中包含一个期望E，此时可以用==stochastic gradient代替 true gradient==
$$
w_{t+1} = w_t + \alpha_t(v_\pi(s_t)- \hat{v}(s_t,w_t))\nabla_w\hat{v}(s_t,w_t)
$$
其中$s_t$是随机变量S的采样，2$\alpha_k$ 用$\alpha_t$替换掉。但此时式中的$v_\pi(s_t)$是未知的，需要用逼近值进行替换。
有两种替换方法

#### 法一：Monte Carlo

用从状态$s_t$出发在整个episode上的discounted return替换$v_\pi(s)$，则
$$
w_{t+1} = w_t + \alpha_t(g_t- \hat{v}(s_t,w_t))\nabla_w\hat{v}(s_t,w_t)
$$

#### 法二：TD learning 

采用TD learning，$r_{t+1} + \gamma \hat{v}(s_{t+1},w_t)$ 就可以看作是$v_\pi(s)$的逼近值，则
$$
w_{t+1} = w_t + \alpha_t(r_{t+1} + \gamma \hat{v}(s_{t+1},w_t)- \hat{v}(s_t,w_t))\nabla_w(s_t,w_t)
$$

### function selection and approximation

如何选取$\hat{v}（s,w） $ 
法一：线性函数逼近（以前广泛使用）
$$
\hat{v}(s,w) = \phi^T(s)w\\
其中\phi^T(s)就是特征向量\\
此时\quad \nabla_w\hat{v}(s,w) = \phi(s)
$$
缺点：很难选出合适的feature vector
优点：相比于非线性形式更容易理解理论特性
		   虽然不能逼近所有函数，但对于例如表格表示这种形式的逼近还是有比较强的表征能力。

法二：使用神经网络作为非线性函数逼近（目前广泛使用）
神经网络的输入是状态state ，输出是参数w

#### 将线性逼近的$\hat{v}（s,w）$ 带入到TD learning的algorithm中，得到

$$
w_{t+1} = w_t + \alpha_t(r_{t+1} + \gamma \hat{v}(s_{t+1},w_t)- \hat{v}(s_t,w_t))\nabla_w\hat{v}(s_t,w_t)\\
\\
\downarrow \\
\\
w_{t+1} = w_t + \alpha_t(r_{t+1} + \gamma \phi^T(s_{t+1})w_t- \phi^T(s_t)w_t)\phi(s_t)
$$

### summary of the story

1.开始于优化目标函数
$J(w) = E[(v_\pi（s) - \hat{v}(S,w) )^2]$ ,表明这实际上是一个policy evaluation问题

2.用随机梯度下降进行优化
$w_{t+1} = w_t + \alpha_t(v_\pi(s_t)- \hat{v}(s_t,w_t))\nabla_w\hat{v}(s_t,w_t)$ 

3.true value 实际上是未知的，所以又用TD learning进行逼近
$w_{t+1} = w_t + \alpha_t(r_{t+1} + \gamma \hat{v}(s_{t+1},w_t)- \hat{v}(s_t,w_t))\nabla_w(s_t,w_t)$ 

### Theoretical analysis  

算法$w_{t+1} = w_t + \alpha_t(r_{t+1} + \gamma \hat{v}(s_{t+1},w_t)- \hat{v}(s_t,w_t))\nabla_w(s_t,w_t)$ 实际上==不能逼近==
$J(w) = E[(v_\pi（s) - \hat{v}(S,w) )^2]$ 

#### 实际上目标函数有多种形式

形式1：true value error
$$
J_E(w) = E[(v_\pi(s) - \hat{v}(S,w) )^2]  = ||\hat{v}(w)- v_\pi||_D^2\\
||x||_D^2 = X^TDX
$$
其中矩阵D的对角元素是状态s的概率分布

形式2：Bellman error
$$
J_E(w) =  ||\hat{v}(w)- \overbrace{(r_\pi + \gamma P_\pi \hat{v}(w))}^{Bellman公式表示v_\pi}||_D^2 = ||\hat{v}(w)- T_\pi(\hat{v}(w))||_D^2\\
T_\pi(\hat{v}(w)) = r_\pi + \gamma P_\pi \hat{v}(w)
$$
形式3：Projected Bellman error
$$
J_{PBE}(w) = ||\hat{v}(w) - MT_\pi(\hat{v}(w))||_D^2\\
M是一个投影矩阵
$$


贝尔曼公式中设计状态转移矩阵，可能会“扭曲”值函数的形式，故由于这种结构原因导致逼近函数与原函数永远无法相等。此时便可以使用投影矩阵M将 $T_\pi(\hat{v}(w))$ 投影回近似函数空间，让其“不再扭曲”.

## 3. Sarsa with function approximation

$$
w_{t+1} = w_t + \alpha_t[r_{t+1} + \gamma \hat{q}(s_{t+1},a_{t+1},w_t) - \hat{q}(s_t,a_t,w_t)\triangledown_w\hat{q}(s_t,a_t,w_t
$$



## 4. Q-learning with function approximation

## 5. Deep Q-learning 

###  basic idea

###  experience replay

###  implementation and example

## 6. Summary