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
w_{t+1} = w_t + \alpha_t[r_{t+1} + \gamma \hat{q}(s_{t+1},a_{t+1},w_t) - \hat{q}(s_t,a_t,w_t)\triangledown_w\hat{q}(s_t,a_t,w_t)
$$

伪代码：
目的：寻找到一个policy可以让agent从特定的s-a对到达target

For  each  episode,  do
	If  the current $s_t$ is not the target , do
    	Take action $a_t$ following $\pi_t(s_t)$ ,generate $r_{t+1}\ ,\ s_{t+1}$ ,then take action $a_{t+1}$ following $\pi_t(s_{t+1})$
		==Value update(parameter upddate)==
$$
w_{t+1} = w_t + \alpha_t[e_{t+1} + \gamma \hat{q}(s_{t+1},a_{t+1},w_t) - \hat{q}(s_t,a_t,w_t)]\triangledown_w\hat{q}(s_t,a_t,w_t)
$$
​		==Policy update==
$$
\pi_{t+1}(a|s_t) = 1 - \frac{\epsilon}{A(s)}(|A(s| - 1 ) \tag{if a = arg $max_{a\in A(s_t)}$ $\hat{q}(s_t,a_t,w_{t+1})$}\\
$$

$$
\pi_{t+1}(a|s_t) = \frac{\epsilon}{A(s)} \tag{otherwise}
$$



## 4. Q-learning with function approximation

定义：
$$
w_{t+1} = w_t + \alpha_t[e_{t+1} + \gamma \underset{a\in A(s_{t+1})}{max} \hat{q}(s_{t+1},a,w_t) - \hat{q}(s_t,a_t,w_t)]\triangledown_w\hat{q}(s_t,a_t,w_t)
$$
伪代码：

For  each  episode,  do
	If  the current $s_t$ is not the target , do
    	Take action $a_t$ following $\pi_t(s_t)$ ,generate $r_{t+1}\ ,\ s_{t+1}$ ,then take action $a_{t+1}$ following $\pi_t(s_{t+1})$
		==Value update(parameter upddate)==
$$
w_{t+1} = w_t + \alpha_t[e_{t+1} + \gamma \underset{a\in A(s_{t+1})}{max} \hat{q}(s_{t+1},a_{t+1},w_t) - \hat{q}(s_t,a_t,w_t)]\triangledown_w\hat{q}(s_t,a_t,w_t)
$$
​		==Policy update==
$$
\pi_{t+1}(a|s_t) = 1 - \frac{\epsilon}{A(s)}(|A(s| - 1 ) \tag{if a = arg $max_{a\in A(s_t)}$ $\hat{q}(s_t,a_t,w_{t+1})$}\\
$$

$$
\pi_{t+1}(a|s_t) = \frac{\epsilon}{A(s)} \tag{otherwise}
$$



## 5. Deep Q-learning 

###  basic idea

deep Q-learning 又叫 Deep Q-network（DQN），是最早的成功将深度神经网络引入强化学习的方法。其中的神经网络充当非线性函数approximator的作用

DQN的目的是最小化目标函数/损失函数(令其为0)，目标函数如下（其实就是q-learning的TD  target）
$$
J(w) = E\big[ (R + \gamma \underset{a\in A(S')}{max} \hat{q}(S',a,w) - \hat{q}(S,A,w))^2 \big]\\
其中（S，A，R，S'）是随机变量
$$
这其实就是贝尔曼最优误差，因为$q(s,a) = E\big[ (R_{t+1} + \gamma \underset{a\in A(S')}{max} \hat{q}(S_{t+1},a) | S_t = s,A_t = a \big]\qquad\forall s,a$  

如何最小化目标函数呢？——梯度下降

但目标函数为w的函数，针对w的梯度，函数中有两个，一个出现在$\hat{q}(s,a,w)$,另一个出现在$y= R + \gamma \underset{a\in A(S')}{max} \hat{q}(S',a,w)$
为了计算的简洁，计算梯度时暂且把y中的w视为常数

具体的，在目标函数中引入两个网络main network——$\hat{q}(S,A,w)$ 和 target network——$\hat{q}(s,a,w_T)$
此时目标函数就可以简化为
$$
J(w) = E\big[ (R + \gamma \underset{a\in A(S')}{max} \hat{q}(S',a,w_T) - \hat{q}(S,A,w))^2 \big]
$$
其中$w_T$是target network的参数，暂且视为常数。

当$w_T$为定值时，目标函数J的梯度可以很容易的表述出来
$$
\triangledown_wJ = E\big[ (R + \gamma \underset{a\in A(S')}{max} \hat{q}(S',a,w_T) - \hat{q}(S,A,w))^2 \ \triangledown_w \hat{q} (S,A,w)\big]
$$

#### DQN中存在一些技巧值得关注。

==技巧一：两个网络，main 和 target==
实现细节：令w和$w_T$初始值相同，在每次迭代时，从replay buffer中选取一个小批次的采样{($s,a,r,s'$)}。main网络的输入就是s和a，目标输出是$y_T = r + \gamma \underset{a\in A(S')}{max} \hat{q}(S',a,w)$  
相当于为了最小化目标函数，让包含变量的网络main去逼近网络target，随后每隔一段时间就更新网络target（令$w_T = w $），循环往复，直到目标函数收敛为0.

==技巧二：experience replay经验回放==
在收集到经验采样后，我们不按照收集到的顺序使用这些采样。反之先将采样存在一个集合，称为replay buffer  $B = {(s,a,r,s')}$ 
随后在每次训练神经网络时，都从replay buffer中拿取一个小批次的采样。
这种采样的抽取方式，被称为经验回放，须遵守均匀分布。因为==在没有先验知识（哪个s-a更重要）的时候，就要一视同仁。== 
这些sample在采集过程中是按照某些确定的策略生成的，为了打破后续样本的关联性，就可以使用经验回放从buffer中均匀选取。

###  implementation and example

伪代码：（此处用的是off policy）
目的：通过由策略$\pi_b$生成的经验样本学习一个最优的target network用来逼近最优的action value
将由策略生成的经验样本存储进replay buffer ——B中。

​	For each iteration,do
​		从B中均匀采样一个mini-batch
​		For each sample(s,a,r,s'),计算其target value 为 $y_T = r + \gamma \underset{a\in A(S')}{max} \hat{q}(s',a,w_T)$  ,其中$w_T$是target network 的不变参数 
​		用mini-batch {(s,a,yT)}来更新main network以最小化目标函数J（w）、

​	每隔c轮迭代，令$w_T = w$  

