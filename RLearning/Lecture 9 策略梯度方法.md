# Lecture Ⅸ Policy Gradient

## 1. Basic idea

在连续函数中，策略可以被表示为$\pi(a|s,\theta)$ 。对于一个神经网络，输入为状态s，输出为采取每个action的概率，其中的参数为$\theta$ 
当状态空间很大时，用表格表示在存储于泛化能力有很低的效率

#### 对于函数表示和表格表示的区别：

1. 定义最优策略方面
   用表格表示时，策略$\pi$能最大化每一个state value即为最优
   用函数表示时，策略$\pi$能最大化具体的标量指标即为最优
2. 获取action的概率方面
   用表格表示，只需要直接查阅表格即可得到action的概率
   用函数表示，需要根据函数结构与参数$\theta$ 来求得概率$\pi$ 
3. 更新策略方面
   用表格表示，直接更新表格中的条目即可更新
   用函数表示，策略不能直接更新，只能通过更新参数$\theta$ 来进行更新。

### policy gradient 的基本思想

首先，用目标函数J$(\theta)$来定义最优策略。随后，采用梯度下降算法来搜寻最优策略。
$$
\theta_{t+1} = \theta_t + \alpha \triangledown_{\theta} J(\theta_t)
$$

## 2. Metric to define optimal policies

==两个指标，average value和average reward==

### ①average state value（average value）

#### 定义

$$
\bar{v_\pi} = \sum_{s\in S}d(s)v_\pi(s)
$$

其中$\bar{v_\pi}$ 是state value的加权平均，d(s) $\geq$ 0 是状态s 的权重
因为$\sum_{s\in S} d(s) = 1$，所以可以把d(s)看作是概率分布，则metric可以改写为$\bar{v_\pi} = E[v_\pi(s)]$ 

向量形式：
$$
\begin{split}
\bar{v_\pi} = \sum_{s\in S}d(s)v_\pi(s) = d^Tv_\pi\\
\\其中
v_\pi = [\ldots v_\pi(s)\ldots]^T \in R^{|S|}\\
\\
d = [\ldots d(s)\ldots]^T \in R^{|S|}
\end{split}
$$

#### 如何选取权重d，分两种情况

#### 1. 权重d对于策略$\pi$是独立无关的，这种情况下，习惯将d标为$d_0$ ，$\bar{v_\pi}$标为$\bar{v}^0_\pi$ 

简单的方式为，将所有状态都视为同样重要，此时$d_0(s) = \frac{1}{|S|}$ 

另一种重要情况是只针对特定状态$s_0$ 。例如，某些任务总是从相同的状态$s_0$ 开始。
此时，$d_0（s_0） = 1 \qquad d_0(s\neq s_0) = 0$

#### 2. 权重d与策略$\pi$ 相关

将d用$d_\pi(s)$来表示，即在$\pi$下的稳态分布，其满足下式，其中$ P_\pi$是状态概率转移矩阵。
$$
d_\pi^T \ P_\pi = d_\pi^T
$$
如果一个状态经常被访问，那么其对应的权重就会高一些；相反的如果很少被访问，那么就会有更低的权重。

### ②average one-step reward（average reward）

#### 定义

$$ {split}
\bar{r}_\pi = \sum_{s\in S}d_\pi(s)r_\pi(s) = E[r_\pi(S)]\\.
\\
其中r_\pi(s)为能获得的一步即时奖励的平均值\\.
\\
r_\pi(s) = \sum_{a \in A}\pi(a|s)r(s,a)\\.
\\
r(s,a) = E[R|s,a] = \sum_rrp(r|s,a)
$$

其中，概率权重d的选取规则与上述一致。

#### 另一种表示方式

$$
\begin{split}
\bar{r}_\pi &= \sum_sd_\pi(s)r_\pi(s)
\\
&=\underset{n \rightarrow \infty}{lim}\frac{1}{n}E\big[ \sum_{k=1}^n R_{t+k}|S_t = s_0\big]
\\
&=\underset{n \rightarrow \infty}{lim}\frac{1}{n}E\big[ R_{t+1} + R_{t+2} + \ldots +R_{t+n}|S_t = s_0\big]
\end{split}
$$

#### 两个指标的补充

1.所有的指标都是$\pi$的函数，而$\pi$是以参数来定$\theta$义的，所以这些指标也就是$\theta$的函数。
不同的$\theta$值也就可以计算出不同的指标值，因此可以通过寻找最优的$\theta$值来最优化指标。
**这就是policy gradient的基本思想**

2.对于这些指标，即可以按照discounted情况定义，也可以通过undiscounted的情况定义。
对于reward，不需要求return，也就不需要考虑discounted case（折扣律），此时就可以有undiscounted case了

3.直觉上来讲，会感觉average reward更加短视。但实际上average reward和average value完全相同。
在discounted情况下，即$\gamma$ < 1 时，有如下结论
$$
\bar{r}_\pi = (1 - \gamma)\bar{v}_\pi
$$

#### 在相关领域的论文中，经常出现的一个指标$J(\theta)$ 实际上跟$\bar{v}_\pi$ 等价，即

$$
J(\theta) = E\big[ \sum_{t=0}^\infty \gamma^t R_{t+1}  \big] = \bar{v}_\pi
$$

## 3. Gradient of the metrics

对于每个metric的梯度表示如下
$$
\begin{split}
\triangledown_\theta \bar{r}_\pi &\approx \sum_{s}d_\pi(s)\sum_{a \in A}\triangledown_\theta \pi(a|s,\theta) q_\pi(a,s) 
\\
\triangledown_\theta \bar{v}_\pi &= \frac{1}{1-\gamma}\triangledown_\theta \bar{r}_\pi
\\
\triangledown_\theta \bar{v}_\pi^0 &= \sum_{s \in S} \rho_\pi(s) \sum_{a \in A}\triangledown_\theta \pi(a|s,\theta) q_\pi(a,s)
\end{split}
$$
上述所有指标（$\bar{v}_\pi^0,\bar{v}_\pi,\bar{r}_\pi$）的梯度都可以用一个**通式**表示。也可以改写成为期望形式,其中S~$\eta$ ，A~$\pi(A|,S,\theta)$    
其中$J(\theta)$可以代表$\bar{v}_\pi^0,\bar{v}_\pi,\bar{r}_\pi$ ; " = "可以表示为严格相等，近似相等或成比例； $\eta$ 是状态s的分布或权重。
==期望形式可以用采样样本逼近梯度==
$$
\begin{split}
\triangledown_\theta J(\theta) &= \sum_{s \in S}\eta(s)\sum_{a \in A}\triangledown_\theta \pi(a|s,\theta) q_\pi(a,s)
\\
&=E\big[ \triangledown_\theta ln\pi(A|S,\theta)q_\pi(S,A)   \big]
\end{split}
$$

#### 证明上述等式相等

$$
易知\quad \triangledown_\theta ln\pi(a|s,\theta) = \frac{\triangledown_\theta \pi(a|s,\theta)}{\pi(a|s,\theta)}(复合函数求导)\\
因此\quad \triangledown_\theta \pi(a|s,\theta)= \pi(a|s,\theta)\triangledown_\theta ln\pi(a|s,\theta)\\
随后便可得到\\
\begin{split}
\triangledown_\theta J &= \sum_{s }d(s)\sum_{a }\triangledown_\theta \pi(a|s,\theta) q_\pi(a,s)\\
&=\sum_{s }d(s)\sum_{a }\pi(a|s,\theta)\triangledown_\theta ln\pi(a|s,\theta) q_\pi(a,s)\\
&=E_{S \sim d}\big[\sum_{a }\pi(a|S,\theta)\triangledown_\theta ln\pi(a|S,\theta) q_\pi(a,S) \big]\\
&=E_{S \sim d,A \sim \pi}\big[\triangledown_\theta ln\pi(A|S,\theta) q_\pi(A,S) \big]\\
&=E\big[ \triangledown_\theta ln\pi(A|S,\theta)q_\pi(S,A)   \big]
\end{split}
$$

#### 补充

因为要计算$ln\pi$  ，所以需要保障对于所有s,a,$\theta$ 都有，$\pi(a|s,\theta)>0$ **(概率虽然不会小于0但是会等于0)**
这可以用softmax函数进行解决，softmax函数可以进行归一化处理，将所有值按比例调整至（-1，1）区间内。
例如，对于一些向量x = [$x_1,\ldots,x_n$]^T
$$
z_i = \frac{e^{x^i}}{\sum_{j=1}^ne^{x^j}}\\
其中z_i \in (0,1) \quad并且\sum_{i=1}^nz_i = 1
$$
由此可以得到策略函数的对应形式
$$
\pi(a|s,\theta) = \frac{e^{h(s,a,\theta)}}{\sum_{a'\in A}e^{h(s,a',\theta)}}\\
其中h(s,a,\theta)为另外的函数
$$
因为对于所有的a对应的策略概率$\pi(a|s,\theta)$都大于0，所以该参数化下的策略是随机化的，具有探索性。

## 4. Gradient-ascent algorithm(REINFORCE)

利用梯度上升使J达到最优的算法如下
$$
\begin{split}
\theta_{t+1} &= \theta_t + \alpha \triangledown_\theta J(\theta)\\
&= \theta_t + \alpha E\big[\triangledown_\theta ln\pi(A|S,\theta_t)q_\pi(S,A)\big]
\end{split}
$$
其中，真实梯度可以被随机梯度替代
$$
\theta_{t+1} = \theta_t + \alpha E\big[\triangledown_\theta ln\pi(a_t|s_t,\theta_t)q_\pi(a_t,s_t)\big]
$$
并且由于$q_\pi$未知，故又可以被近似为
$$
\theta_{t+1} = \theta_t + \alpha E\big[\triangledown_\theta ln\pi(a_t|s_t,\theta_t)q_t(a_t,s_t)\big]
$$

#### 有几种不同的方式可以逼近$q_\pi$ ，本章中采用基于Monte Carlo方法，也称REINFORCE

### 补充

#### 1. 如何采样

$E_{S \sim d,A \sim \pi}\big[\triangledown_\theta ln\pi(A|S,\theta) q_\pi(A,S) \big] \rightarrow \triangledown_\theta ln\pi(a_t|s_t,\theta_t)q_\pi(a_t,s_t)$ 

S的采样是服从于分布d的，d 是在策略$\pi$下长期运行得到的分布
A的采样是服从于策略$\pi(A|S,\theta)$的，因此，$a_t$应该是在遵循策略$\pi(\theta_t)$所对应的状态$s_t$采样得到的。
又由于采样得到的值反过来用来更新策略，所以**policy gradient是on-policy的**。



![image-20250911162722362](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250911162722362.png)

#### 伪代码

初始化：参数化函数$\pi(a|s,\theta)$ ,$\gamma \in (0,1)$ , $\alpha > 0$ . 
目的：寻找能最大化目标函数J的策略

For the $k$th iteration, do
	选择状态 $s_0$ 并遵循策略 $\pi(\theta_k)$  计算出一个episode，假设episode 为 {$s_0,a_0,r_1,\ldots,s_{T-1},r_T$} 
	For t = 0,1,...,T-1, do
		Value update : $q_t(s_t,a_t) = \sum_{k=t+1}^T \gamma^{k-t-1}r_k$ 
		Policy update : $\theta_{t+1} = \theta_t + \alpha\triangledown_{\theta} ln\pi(a_t|,s_t,\theta_t)q_t(s_t,a_t)$  
	$\theta_k = \theta_T$ 

没有立刻用新数据采集，因为蒙特卡洛是offline的，需要跑完episode才能更新

