# Lecture Ⅴ Monte Carlo Learning

## 1. Motivating  example

#### Monte Carlo estimation

对于一个投掷硬币的例子，硬币正面记为x= 1，硬币反面记为x= -1，随机变量x的汇总为X。则求取X的期望EX有两种方法。

#### 法一：model-based

$$
p(X = 1) =0.5\qquad p(X = -1) = 0.5 \\
EX = 1*p(X = 1)+(-1)*p(X = -1) = 0
$$

#### 法二：model-free

$$
收集投掷硬币n次的样本数据 X_1-X_n
\\EX = \frac{1}{n}\sum_{j=1}^{n}{X_j}
$$

#### 大数定理：

对于一个随机变量X，其对应的采样值${x_j}_{j = 1}^N$ 是独立同分布的，令$\frac{1}{N}\sum_{j=1}^{N}{x_j}$ 作为采样均值。则有
$$
E[\overline{x}] = E[X]\\
Var[\overline{x}] = \frac{1}{N}Var[X]
$$
因此，$\overline{x}$ 是E[X]的无偏估计，并且其方差在N趋向无穷时趋于0.

## 2. The simplest MC-based RL  algorithm

#### Algorithm : MC Basic  关键：如何将policy iteration algorithm替换成model-free类型的

$$
\pi_{k+1}(s) = arg \underset{\pi}{max}\sum_{a}\pi(a|s)q_{\pi k }(s,a)\qquad s\in S
$$

policy iteration中，与模型（概率）相关的是PE部分，其中与模型相关的量是action value  $q_{\pi k }(s,a)$ ,所以关键就是如何用model-free替代该模型概率

### 求解action value的两种方法

法（1）需要模型，法（2）不需要模型。凭借方法（2），可以依靠data（samples or experiences）计算action value
$$
\begin{align*}
&q_{\pi k }(s,a) = \sum_{r}{p(r|s,a)r}+ \gamma \sum_{r'}{p(s'|s,a)v_{\pi k}(s')} \tag{1}   \\
&q_{\pi k }(s,a) = E[G_t|S_t = s ,A_t = a] \tag {2}
\end{align*}
$$

###  The  procedure  of  Monte  Carlo  estimation  of  action  value

从一个状态（s,a）出发，遵循policy   $\pi_k$  ，运行后得到一个episode（==trajectory==）

返回的episode就是discounted return——g(s,a),其代表$G_t$ 的采样

假设现有一组采样episodes，因此可以得到{$g_{(j)}(s,a)$}

而后便可得到$q_{\pi k }(s,a) = E[G_t|S_t = s ,A_t = a] \approx \frac{1}{N}\sum_{i =1}^N g_{(i)}(s,a) $

#### 基本思想：无模型时，便可使用data（即sample，强化学习中称作experience--==exp==）

### MC  Basic  algorithm

步骤一：policy evaluation

该步骤为了获取所有(s,a)对应的action value，对于每一个(s,a)，运行无数次（或足够次数）的episodes后，所得到的return的平均值就用来估计action value

步骤二：policy improvement

该步骤为了求解策略$\pi_{k+1}$  ,该步骤与策略迭代的第二步PI完全相同

MCB与policy iteration之间的区别只在于第一步PE，MCB通过data求解action value，policy iteration 求解state value。

求解$v_{\pi_{k}}$实际上等同于求解$q_{\pi_{k}}$   

### 伪代码

Initial guess $\pi_0$

While 估计值尚未收敛，for the kth iteration,do 

​		For every state s$\in$ S ,do 

​				For evevy action a $\in$ A, do 

​						遵循策略 $\pi_k$ 从(s,a)中收集足够多的批次（trajectory）

​						进行MC-Based evaluation ： $q_{\pi_k }(s,a)$ = average return

​				Policy improvement step

​				$a_k^*(s) = arg\ \underset{a}{max} q_{\pi_k}(s,a) $

​				$\pi_{k+1}(a|s) = 1 \quad if a = a_k^*,\quad \pi_{k+1}(a|s) = 0 \ otherwise$

MCB是基于policy iteration的一个变形

MCB的efficiency很低，但是揭示了model-free的底层原理

MCB直接估计了action value ，因为action value到state value需要模型，MCB无法求得（但是action 和state在作用是一致的）

## 3. Use  data and value estimate more efficiency

#### Algorithm : MC Exploring Starts(强制充分探索)

### Use   data   more  efficiently

考虑一个回合（episode）$s_1 \xrightarrow{a_2} s_2 \xrightarrow{a_4} s_1 \xrightarrow{a_2} s_2 \xrightarrow{a_3} s_5 \xrightarrow{a_1} \ldots$  

==Visit== :在一个episode中一个state-action对的每次出现，都叫做对这个state-value对的visit

#### first-visit method：一段episode只用来计算起始state-value==（后续都写作s-v对）==对的$q_\pi(s_0,a_0)$ ,对于episode后续state-value对										的$q_\pi(s,a)$ 都没有被存储。

first-visit：相当于有效样本仅仅是开始访问的（s，a）

#### every-visit method：一段episode中每一个s-t对所对应的$q_\pi(s,a)$ 都会被存储起来用于后续的value estimate

every-visit：相当于在当前样本中涉及的所有以任意（s，a）为起始的discounted return都可以当做有效样本利用

#### 该方法旨在确保所有s-t对对应的action value 都有作为episode的初始s-t的机会，但是这种方法需要考虑一个episode中可能出现多次相同的s-t对，这时要考虑对应多个action value的相关性，因为MC learning都是基于大数定理的独立同分布性质。

（此时相同的s-t对相隔越远其相关性越差，一个s-t其对应的action value对其周围的s-t对具有依赖性）

### Use value estimate more efficiently

对于MC-base 更新策略有两种方法

#### 法一：在一个policy evaluation的步骤中，先收集一个s-t对里所有的episodes然后再求均值用来近似action value

这种方法在MC-based中被采用。其缺点在于需要等到所有episodes都被收集完成后agent才能开始计算

#### 法二：agent先采用返回的单一episode估计action value，随后在每次新的episode被添加时对估计的action value进行迭代更新

此方法类似于truncated policy iteration方法，估计的action value最后是会收敛的。

### Generaliazed policy iteration（GPI）

GPI不是一个具体的算法，他是指在PE和PI之间转换迭代的过程的一个idea或framework，许多model-based、model-free的RL算法都属于这个框架。

### 伪代码

![](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250822160527227.png)

#### 这里面有一个策略改进==逆序计算==，对于一个0-t步的episode，其discounted return计算如下

$$
g_0 = r_0 + \gamma r_1 + \gamma^2 r_2 + \ldots = r_0 + \gamma g_1 \\
g_1 = r_1 + \gamma r_2 + \gamma^2 r_3 + \ldots = r_1 + \gamma g_2\\
\ldots \\ 
g_t = r_t
$$

其每一步的reward都是已知的，倘若顺序计算discounted return 其计算复杂度则为$O(n^2)$ ,逆序计算则不包含重复计算量，复杂度为$O(n)$ 

#### 实际中MC Exploring Starts 也是无法使用的方法，因为在与环境交互的情况中，很难满足其强制探索所有初始情况的条件。

## 4. MC without exploring starts

#### Algorithm : MC   $\epsilon$ -Greedy 

### soft policy：

如果一个策略在确定action时，任何一个action都有可能被选取，那么这个策略可以被称为soft  policy

greedy  policy属于deterministic，soft  policy属于stochastic

对于一个soft policy，一些足够长的episodes在visit足够多的次数后便可以遍历所有的s-t对。

#### 遍历所有的s-t对是对的，但是exploring starts由于用了greedy policy，想要确定visit所有的s-t对就必须要从初始s-t下手。

### $\epsilon$ - greedy  policies (soft  policy 的一种)

$$
\pi(a|s) = \begin{cases} 1-\frac{\epsilon}{\vert A(s) - 1 \vert}(\vert A(s)\vert - 1 )\qquad{for\ the\ greedy \ action}
\\
\frac{\epsilon}{\vert A(s)\vert} \qquad\qquad\qquad\quad\qquad{for\ the\ other\ |A(s)|-1\ actions)} \end{cases}
$$

其中$\epsilon \in [0,1]$,    |A(s)|是状态s对应action的数量。

#### 其中选择greedy action的概率总是要远大于选择其他actions，因为$1- \frac{\epsilon}{|A(s|}(|A(S)|-1) = 1-\epsilon +\frac{\epsilon}{|A(s)|}\geq \frac{\epsilon}{|A(s)|}$ 

#### 选择$\epsilon$ - greedy  policies的理由：平衡了  exploitation和 exploration

  exploitation：剥削 exploration：探索

当$\epsilon $ = 0，就变成了greedy policy，有更少的探索更多的剥削

当$\epsilon =1 $,就变成了均匀分布，更多的探索更少的剥削

### MC $\epsilon$ -Greedy algorithm

![image-20250822164105649](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250822164105649.png)

![image-20250822164116834](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250822164116834.png) 

防止浪费，用every'-visit