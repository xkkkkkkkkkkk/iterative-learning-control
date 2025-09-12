# Lecture  Ⅶ  Temporal  Difference  Learning

## 1. Motivating examples

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093713473.png" alt="image-20250827093713473" style="zoom:80%;" />

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093823428.png" alt="image-20250827093823428" style="zoom:80%;" />

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093931845.png" alt="image-20250827093931845" style="zoom:80%;" />

## 2. TD learning of state values

#### 该小节所指的TD算法就是特定用于在给定policy求解state value的算法，常规下TD算法指一类RL算法

算法需要的data/experience：通过策略$\pi$ 产生的（$s_0,r_1,s_1,\ldots,s_t,r_{t+1},s_{t+1}$）

### TD algorithm:

第二个式子常规下默认为隐式。其意义更新时只更新当前采用的状态对应的state value。
$$
\begin{split}
& \overbrace{v_{t+1}（s_t)}^{新估计\ new\ estimate} = \overbrace{v_t(s_t)}^{旧估计\ current\ estimate}-\alpha_t(s_t)\big[ \overbrace {v_t(s_t)-\underbrace{[r_{t+1} + \gamma v_t(s_{t+1}}_{TD\ target\ \bar v_t})}^{TD\ error\ \delta_t}]  \big]\\
\\
& v_{t+1}(s) = v_t(s) \qquad  \forall s\neq s_t\\
\end{split}
$$

### TD target

整个TD算法就是为了将$v(s_t)$朝着$\bar v_t$ 逼近
$$
\begin{split}
& v_{t+1}（s_t) = v_t(s_t)-\alpha_t(s_t)\big[ v_t(s_t)-\bar v_t  \big]\\
& v_{t+1}（s_t) - \bar v_t = v_t(s_t)-\bar v_t - \alpha_t(s_t)\big[ v_t(s_t)-\bar v_t  \big]\\
& 即\\
& v_{t+1}（s_t) - \bar v_t = (1 - \alpha_t(s_t))(v_t(s_t)-\bar v_t)\\
& 因为0<(1 - \alpha_t(s_t))<1\\
& 所以v_{t+1}(s_t) 比 v_t(s_t)更接近\bar v_t
\end{split}
$$

### TD error

是两个不同时刻之间的差值（所以叫时序差分），反映出了当前时刻估计的 $v_t$ 和 整体策略要估计的$ v_\pi$ 
$$
\delta_t = v(s_t) - [r_{t+1}+ \gamma v(s_{t+1})]
$$
由TD error可以得出
$$
\begin{split}
& \delta_{\pi,t} = v_\pi (s_t) - [r_{t+1} + \gamma v_\pi(s_{t+1})]\\
& 由此可得\\
& E[\delta_{\pi,t}|S_t = s_t] = v_\pi(s_t) - E[R_{t+1} + \gamma v_\pi(S_{t+1})|S_t=s_t] = 0(由贝尔曼公式可以得出)
\end {split}
$$
所有TD error 用来描述利用当前data求出的state value与策略预期的state value之间的误差

### TD algorithm的理念

TD算法的意义：求解无模型下的Bellman equation

策略$\pi$ 对应的state value为==(Bellman expectation equation)==
$$
\begin{split}
& v_\pi(s) = E[R +\gamma G|S = s]\qquad s\in S\\
& S'为状态s对应的下一个状态，则上式可以改写为\\
& v_\pi(s) = E[R + \gamma v_\pi(S')|S = s]\qquad s\in S
\end{split}
$$
求解上述的贝尔曼公式就需要用到RM algorithm，定义
$$
\begin{split}
& \overbrace{g(v(s))}^{g(w)} = v(s) - E[R+\gamma v_\pi(s')|s] = 0\\
\\
& 上式也可以改写为\\
\\
& g(v(s)) = 0
\end{split}
$$

## 3. TD learning of action values : Sarsa

要改进策略就要求出action value，Sarsa就可以直接求得action value

任务：在给定策略下利用experience求得action value。已知 experiences{$s_t,a_t,r_{t+1},s_{t+1},a_{t+1}$}

### Sarsa algorithm:（Policy evaluation）

$$
\begin{split}
&q_{t+1}(s_t,a_t) = q_t(w_t,a_t)-\alpha (s_t,a_t)\big[ q_t(s_t,a_t) - [r_{t+1}+ \gamma q_t(s_{t+1,a_{t+1}})]  \big]\\
&q_{t+1}(s,a) = q_t(s,a) \qquad \forall (s,a)\neq (s_t,a_t)
\end{split}\\
\\
q_t(s_t,a_t)就是q_\pi(s_t,a_t)在t时刻的估计值\\
\alpha(s_t,a_t)是取决于s_t,a_t的学习率
$$

#### 对比于TD Leaning  除了将state value（v) 替换成了action value（a）其余皆相同

#### Sarsa就是state-action-reward-state-action的缩写

#### Sarsa的收敛性也于TD算法完全一样，定理也是



#### 最终的目标是求解最优策略，所以将Sarsa algorithm与Policy Improvement相结合，其伪代码如下

​		For each episode，do
​				If the current s_t is not target state, do 
​				收集experiences$（s_t,a_t,r_{t+1},s_{t+1},a_{t+1}）$ 
​				遵循策略$\pi_t(s_t)$  采取action  $a_t$  由此得到$r_{t+1},s_{t+1}$  ,再遵循新策略  $\pi_t(s_t+1)$ 采取action 	$a_{t+1}$ 	
​				Update q-value：
​						$q_{t+1}(s_t,a_t) = q_t(s_t,a_t)-\alpha_t (s_t,a_t)\big[ q_t(s_t,a_t) - [r_{t+1}+ \gamma q_t(s_{t+1,a_{t+1}})]  $ 

​				Update policy:(==|A|是所有action的数量==)

​						$\pi_{t+1}(a|s_t) = 1- \frac{\epsilon}{|A|}(|A|-1)\qquad if\ a = arg\ \underset{a}{max}q_{t+1}(s_t,a)\\ \pi_{t+1}(a|s_T) = \frac{\epsilon}{|A|}\qquad otherwise$

####  注意

①$s_t$ 的策略实在得到action value  $q(a_t,s_t)$  立刻更新的，即是on-policy的

②策略用的是ε-greedy策略，以便更好平衡探索——剥削

③算法核心思想：求解给定策略下的贝尔曼公式，去高效率的找到最优策略。

==Sarsa算法求解的是从特定起始状态出发到目标状态的最优策略==
这意味着其不同于之前那些求解全局最优策略（初始状态不唯一，包括所有情况）的算法

example：$t_{target} = 0,r_{forbidden} = r_{boundary} = -10,r_{other} = -1\\\ learning\ rate\ \alpha = 0.1\qquad \epsilon = 0.1$ 

起始位置：左上角

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250828102712589.png" alt="image-20250828102712589" style="zoom:67%;" />



## 4. TD learning of action values: Expected Sarsa

### Expected  Sarsa algorithm:

$$
\begin{split}
& q_{t+1}(s_t,a_t) = q_t(w_t,a_t)-\alpha (s_t,a_t)\big[ q_t(s_t,a_t) - [r_{t+1}+ \gamma E[q_t(s_{t+1},A)]] \\
\\
& q_{t+1}(s,a) = q_t(s,a)\qquad \forall(s,a) \neq (s_t,a_t)\\
\\
& where\\
\\
& E[q_t(s_{t+1},A)] = \sum_a{\pi_t(a|s_{t+1})q_t(s_{t+1,a})} = v_t(s_{t+1})\\
\\
& 是在策略\pi_t下q_t(s_{}t+1,a）的value期望
\end{split}
$$

#### 与Sarsa比较：TD target从$r_{t+1}+ \gamma q_t(s_{t+1,a_{t+1}})$ 变成了$r_{t+1}+ \gamma E[q_t(s_{t+1},A)$ 

需要的experience（data）从s，a，r变回了s，a。变量变少导致==随机性降低==，与Sarsa相比估计的方差较低。

与此同时，参数变少导致需要计算期望，计算量变大。

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250828135123788.png" alt="image-20250828135123788" style="zoom:67%;" />

## 5. TD learning of action values : n-step Sarsa 

### n-step Sarsa:

可以整合 Sarsa和Monte Carlo的算法

#### 对于action value的定义如下

$$
q_\pi(s,a) = E[G_t|S_t = s,A_t = a]\\
.\\
对应的\ discounted\ return\ G_t可以被分解成如下形式\\
$$

$$
\begin{split}
Sarsa \leftarrow& G_t^{(1)} = R_{t+1}+\gamma q_\pi(S_{t+1},A_{t+1})\\
& G_t^{(2)} = R_{t+1}+\gamma R_{t+2}+\gamma^2 q_\pi(S_{t+2},A_{t+2})\\.\\.\\.\\
n-step\ Sarsa\leftarrow   &G_t^{(n)} = R_{t+1}+\gamma R_{t+2} +\ldots +\gamma ^nq_\pi(S_{t+n},A_{t+n})\\.\\.\\.\\
MC \leftarrow &G_t^{(\infty)} = R_{t+1}+\gamma R_{t+2}+\gamma^2R_{t+3}+\ldots
\end{split}
$$

其中$G_t=G_t^{(1)}= G_t^{(2)} = G_t^{(n)}=  G_t^{(\infty)}$ 

由上式可知
Sarsa aims to solve
$$
q_\pi(s,a)= E[G_t^{(1)}|s,a] = E[R_{t+1}+\gamma q_\pi(S_{t+1},A_{t+1})|s,a]
$$
MC learning aims to solve 
$$
q_\pi(s,a)= E[G_t^{(\infty)}|s,a] = E[R_{t+1}+\gamma R_{t+2} + \gamma^2 R_{t+3}+ \ldots|s,a]
$$
n-step Sarsa aims to solve 
$$
q_\pi(s,a)= E[G_t^{(n)}|s,a] = E[R_{t+1}+\gamma R_{t+2}+\ldots+\gamma ^n q_\pi(S_{t+n},A_{t+n})|s,a]
$$

#### n-step Sarsa algorithm具体算法为：

$$
q_{t+1}(s_t,a_t) = q_t(s_t,a_t)-\alpha_t(s_t,a_t)\big[q_t(s_t,a_t)-[r_{t+1}+\gamma r_{t+2}+\ldots+\gamma ^n q_t(s_{t+n},a_{t+n}]\big]
$$

### n-step Sarsa 性质

#### n-step Sarsa 需要的experience有 $(s_t,a_t,r_{t+1},s_{t+1},s_{t+1},\ldots,r_{t+n},s_{t+n},a_{t+n})$   

因为n-step Sarsa需要的data不能在t时刻全部得到，所以不能即刻进行更新，需要像MC learning那样等，但区别于MC learning等到所有data得出才开始计算，n-step Sarsa 只需要等到t+n时刻。

==当n很大时==，n-step Sarsa 的性质就更贴近于MC learning，因此算法就有更大的方差以及更小的偏差

==当n很小时==，n-step Sarsa 的性质就更贴近于Sarsa，因此算法就有着更大的偏差以及更小的方差

## 6. TD learning of optimal action values : Q-learning

Q-learning 直接估计最优策略，所以不需要PE和PI交替迭代

###  Q-learning algorithm:

$$
\begin{split}
& q_{t+1}(s_t,a_t) = q_t(s_t,a_t) - \alpha(s_t,a_t)\big[ q_t(s_t,a_t) - [r_{t+1} + \gamma \underset{a\in A}{max}q_t(s_{t+1},a)]  \big] \\
& q_{t+1}(s,a) = q_t(s,a) \qquad \forall (s,a) \neq (s_t,a_t)
\end{split}
$$

Q-learning 和 Sarsa 非常相似，唯一区别在于二者的TD target

Q-learning的TD target为$r_{t+1} + \gamma \underset{a\in A}{max}q_t(s_{t+1},a)$ 
Sarsa的TD target 为$r_{t+1} + \gamma q_t(s_{t+1},a_{t+1})$

#### Q-learning 不同于前面求解贝尔曼方程，变为了求解一个贝尔曼最优方程

$$
q(s,a) = E[R_{t+1}+ \gamma \underset{a}{max}q(S_{t+1},a)|S_t = s,A_t = a]\qquad \forall s,a
$$

### Off-policy vs on-policy

#### 强化学习中存在两个策略：behavior policy 和 target policy

behavior policy：和环境进行交互，生成experience

target policy：持续更新，最后得到最优策略

#### 基于这两种策略可以定义两种算法：on-policy 和 off-policy

on-policy ：behavior policy和target policy相同

off-policy ：behavior policy和target policy不同

#### Off-policy的优点：

可以依靠其他策略生成的experience对最优策略进行搜索。例子：当想兼顾探索性与最优策略时，behavior就可以选择探索性更强的policy，与此同时target policy也可以专注于寻找最优策略。

#### 如何判断一个算法是on-policy还是off-policy

一、算法在数学上解决什么数学问题
二、算法需要什么数据来让算法运行

==例子：==

#### on-policy ：$ 给定策略\pi_t \rightarrow exp \rightarrow q_{\pi_t} \rightarrow \pi_{t+1}$ 

![image-20250828153354425](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250828153354425.png)

![image-20250828153521354](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250828153521354.png)

![image-20250828153952891](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250828153952891.png)

## 7. Q-learning pseudo code and examples

### Q-learning 伪代码

#### （on-policy 版本）

For each episode，do
				If the current s_t is not target state, do 
				收集experiences$（s_t,a_t,r_{t+1},s_{t+1}）$ 
				遵循策略$\pi_t(s_t)$  采取action  $a_t$  由此得到$r_{t+1},s_{t+1}$  
				Update q-value：
						$q_{t+1}(s_t,a_t) = q_t(s_t,a_t)-\alpha_t (s_t,a_t)\big[ q_t(s_t,a_t) - [r_{t+1}+ \gamma \underset{a}{max}\  q_t(s_{t+1,})]  $ 

​				Update target policy:(==|A|是所有action的数量==)

​						$\pi_{t+1}(a|s_t) = 1- \frac{\epsilon}{|A|}(|A|-1)\qquad if\ a = arg\ \underset{a}{max}q_{t+1}(s_t,a)\\ \pi_{t+1}(a|s_T) = \frac{\epsilon}{|A|}\qquad otherwise$

#### (off-policy 版本)

 For each episode{$s_0,a_0,r_1,s_1，a_1,r_2，a_1,r_2$} generated by $\pi_b$(==behavior policy==)，do
 		For each step t = 0,1,2...of the episode ,do

​				Update q-value:
​						$q_{t+1}(s_t,a_t) = q_t(s_t,a_t)-\alpha_t (s_t,a_t)\big[ q_t(s_t,a_t) - [r_{t+1}+ \gamma \underset{a}{max}\  q_t(s_{t+1,})]  $ 

​				Update target policy:(	$\pi_{T,t+1}$ ==指target policy==)

​						$\pi_{T,t+1}(a|s_t) = 1\qquad if\ a = arg\ \underset{a}{max}q_{t+1}(s_t,a)\\ \pi_{T,t+1}(a|s_T) = 0\qquad otherwise$



## 8. Unified point of view 

本章所有的算法都可以用一个通式表示
$$
q_{t+1}(s_t,a_t) = q_t(s_t,a_t) - \alpha_t(s_t,a_t)[q_t(s_t,a_t)-\bar{q}_t]\\
.\\
\bar{q}_t 是TD\ target
$$
不同的TＤ算法有不同的$\bar{q}_t$

| Algorithm      | Expression of $\bar{q}_t$                                    |
| -------------- | ------------------------------------------------------------ |
| Sarsa          | $\bar{q}_t = r_{t+1} + \gamma q_t(s_{t+1},a_{t+1})$          |
| n-step Sarsa   | $\bar{q}_t = r_{t+1} + \gamma r_{t+2} + \ldots + \gamma^n q_t(s_{t+n},a_{t+n})$ |
| Expected Sarsa | $\bar{q}_t = r_{t+1} + \gamma \sum_a{\pi_t(a|s_{t+1}q_t(s_{t+1},a))}$ |
| Q-learning     | $\bar{q}_t = r_{t+1} + \gamma\ \underset{a}{max}\ q_t(s_{t+1},a)$ |
| Monte Carlo    | $\bar{q}_t = r_{t+1} +r_{t+2} +\ldots$                       |

当$\alpha_t(s_t,a_t) = 1$时，$q_{t+1}(s_t,a_t) = \bar{q}_t$ 。MC algorithm也可以表示为统一形式

#### 所有的算法都可以被视为用于求解BE 或BOE 的随机逼近算法

| Algorithm      | Equation aimed to solve                                      |
| -------------- | ------------------------------------------------------------ |
| Sarsa          | BE：$q_\pi(s,a) = E[R_{t+1}+\gamma q_\pi(S_{t+1},A_{t+1})|S_t=s,A_t = a]$ |
| n-step Sarsa   | BE：$q_\pi(s,a) = E[R_{t+1}+\gamma R_{t+2}+\ldots+ \gamma^n q_\pi(S_{t+n},A_{t+n})|S_t=s,A_t = a]$ |
| Expected Sarsa | BE：$q_\pi(s,a) = E\big[R_{t+1}+\gamma E_{A_{t+1}}[q_\pi(S_{t+1},A_{t+1})]|S_t=s,A_t = a\big]$ |
| Q-learning     | BOE：$q_\pi(s,a) = E[R_{t+1}+\underset{a}{max} \ q(S_{t+1},a)|S_t=s,A_t = a]$ |
| Monte Carlo    | BE：$q_\pi(s,a) = E[R_{t+1}+\gamma R_{t+2}+\ldots|S_t=s,A_t = a]$ |

#### 这些TD 方法本质上来说是求解给定策略的贝尔曼公式，对于如何搜索一个最优策略，就是把PE和PI相结合就可以实现。







