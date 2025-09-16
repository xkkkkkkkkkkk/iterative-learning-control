

# Lecture Ⅹ Actor-Critic Method

## 1. The  simplest actor- critic(QAC)

用Monte Carlo方法估计$q_t(s_t,a_t)$ 即为上一章所讲的REINFORCE，而用temporal-difference（TD）learning方法估计$q_t(s_t,a_t)$ 的即为actor-critic。

#### 伪代码

初始化：$\theta_0$为初始参数的策略$\pi(a|s,\theta_0)$ , $w_0$为初始参数的值函数q(s,a,$w_0$) ,w和$\theta$ 的学习率都大于0。
目标：找到能够优化目标函数的最优策略
At time step t in each episode,do
		通过策略$\pi(a|s_t,\theta_t)$生成action $a_t$ ，遵循action得到 $r_{t+1}, s_{t+1}$。再通过策略$\pi(a|s_{t+1},\theta_t)$生成action  $a_{t+1}$ 
		Critic(value update)
				$w_{t+1} = w_t + \alpha_w[r_{t+1} + \gamma q(s_{t+1,a_{t+1},w_t}) - q(s_t,a_t,w_t)]\triangledown_w q(s_t,a_t,w_t)$ 
		Actor(Policy update)
				$\theta_{t+1} = \theta_t + \alpha_\theta \triangledown_\theta ln\pi(a_t|s_t,\theta_t)q(s_t,q_t,w_{t+1})$ 

#### 该策略是on-policy的，因为策略具有随机性，不需要使用ε-greedy等技巧。

## 2. Advantage actor-critic(A2C)

### ==Baseline invariance==

#### 基本思想：引入偏差量对梯度无影响

$$
\begin{split}
\triangledown_\theta J(\theta) &= E_{S \sim \eta,A\sim \pi}[\triangledown_\theta ln\pi(A|S,\theta_t)q_\pi(S,A)]
\\
&=E_{S \sim \eta,A\sim \pi}[\triangledown_\theta ln\pi(A|S,\theta_t)q_\pi(S,A) - b(S)]
\\
&b(S)是S的标量函数
\end{split}
$$

**证明$E_{S \sim \eta,A\sim \pi}[\triangledown_\theta ln\pi(A|S,\theta_t)b(S)] = 0$**  
$$
\begin{split}
E_{S \sim \eta,A\sim \pi}[\triangledown_\theta ln\pi(A|S,\theta_t)b(S)] &= \sum_{s\in S}\eta(s)\sum_{a\in A}\pi(a|s,\theta_t)\triangledown_\theta ln\pi(a|s,\theta_t)b(s)
\\
&= \sum_{s\in S}\eta(s)\sum_{a\in A}\triangledown_\theta \pi(a|s,\theta_t)b(s)
\\
&= \sum_{s\in S}\eta(s)b(s)\sum_{a\in A}\triangledown_\theta \pi(a|s,\theta_t)
\\
&= \sum_{s\in S}\eta(s)b(s)\sum_{a\in A}\triangledown_\theta 1 =0
\end{split}
$$

**虽然b(S)不影响E[X],但是会影响方差va(X)**。如下式，当b很大时，方差受影响也很大。
$$
tr[var(X)] = E[X^tX] - \bar{x}^T\bar{x}
\\
\begin{split}
E[X^TX] &= E[(\triangledown_\theta ln\pi)^T(\triangledown_\theta ln\pi)(q(S,A) - b(s))^2]
\\
&= E[||\triangledown_\theta ln\pi||^2(q(S,A) - b(s))^2]
\end{split}
$$
目标：找到一个最优偏差b以最小化方差var(X)
优点：用随机采样来逼近E[X]，估计的误差也会很小
在REINFORCE和QAC中，没有偏差b存在，或者说b=0，者不能保证是一个好的偏差。

对于任意s来说，能够最小化方差的最优偏差b为
$$
b^*(s) = \frac{E_{A\sim\pi}[||\triangledown_\theta ln\pi(A|s,\theta_t)||^2q(s,A)]}{E_{A\sim\pi}[||\triangledown_\theta ln\pi(A|s,\theta_t)||^2}
$$
虽然这个偏差b是最优的，但是计算过于复杂，所以实际中通常将$||\triangledown_\theta ln\pi(A|s,\theta_t)||^2$ 去掉，选择次优的偏差b,这也是s的state value
$$
b(s) = E_{A\sim \pi}[q(s,A)] = v_\pi(s)
$$

### The algorithm of advantage actor-critic

梯度上升算法为==（也被叫做优势函数）==
$$
\begin{split}
\theta_{t+1} &= \theta_t + \alpha E\big[\triangledown_\theta ln\pi(A|S,\theta_t)[q_\pi(S,A)-v_\pi(S)]\big]
\\
&= \theta_t + \alpha E\big[\triangledown_\theta ln\pi(A|S,\theta_t)\delta_\pi(S,A) \big]
\end{split}
\\
其中
\\
\delta_\pi(S,A) = q_\pi(S,A) - v_\pi(S)
$$
此外，算法还可以表述为另外格式.令步长与相对值δ成正比，而不是绝对值q成正比更合理。该算法仍可以平衡探索与利用。
$$
\begin{split}
\theta_{t+1} &= \theta_t +\alpha \triangledown_\theta ln \pi(a_t|s_t, \theta_t) \delta_t(s_t,a_t)
\\
&=\theta_t +\alpha \frac{\triangledown_\theta  \pi(a_t|s_t, \theta_t)}{\pi(a_t|s_t, \theta_t)} \delta_t(s_t,a_t)
\\
&= \theta_t +\alpha \underbrace{\Big(\frac{\delta_t(s_t,a_t)}{\pi(a_t|s_t, \theta_t)}\Big)} _{step \ size}\triangledown_\theta  \pi(a_t|s_t, \theta_t)
\end{split}
$$
此外，优势函数还可以由TD 误差进行逼近
优点：只需要一个网络就可以逼近$v_\pi(s)$，而不需要两个网络去计算$q_\pi(s,a)$ 和 $v_\pi(s)$ 
$$
\delta_t = q_t(s_t,a_t) - v_t(s_t) \rightarrow r_{t+1} + \gamma v_t(s_{t+1}) - v_t(s_t)
$$

#### 伪代码：

目标：找到优化J(θ)的最优策略
At time step t in each episode , do
		通过策略$\pi(a|s_t,\theta_t)$生成action $a_t$ ，遵循action得到 $r_{t+1}, s_{t+1}$。
		TD error(advantage function)
				$\delta_t = r_{t+1} + \gamma v_t(s_{t+1},w_t) - v_t(s_t,w_t)$
		Critic(value update)
				$w_{t+1} = w_t + \alpha_w\delta_t\triangledown_wv(s_t,w_t)$
		Actoe(policy update)
				$\theta_{t+1} = \theta_t + \alpha_\theta\delta_t\triangledown_\theta ln\pi(a_t|s_t,\theta_t)$ 
**该方法为on-policy，因为策略$\pi(\theta_t)$ 为随机的，不需要再另外使用ε- greedy等策略。**

## 3. Off-policy actor-critic

#### ==重要性采样==

off-policy 决定了采样和逼近的策略不一样，对应的采样服从的分布为采样策略的分布，应用到逼近策略时本应该服从逼近策略的分布。

重要性采样的目的就是为了服从$p_1$的采样x来估计服从$p_0$的期望。
$$
E_{X\sim p_0}[X] = \sum_x p_0(x)x = \sum_x p_1(x)\underbrace{\frac{p_0(x)}{p_1(x)}x}_{f(x)} = E_{X\sim p_1}[f(X)]
$$
经由上式，便可以经计算$ E_{X\sim p_1}[f(X)]$ 来估计$ E_{X\sim p_0}[X]$ 。对于估算$ E_{X\sim p_1}[f(X)]$，只需令
$$
\bar{f} = \frac{1}{n}\sum_{i=1}^nf(x_i)\qquad  其中x_i \sim p_1
\\那么就有
\\
 E_{X\sim p_1}[\bar{f}] =  E_{X\sim p_1}[f(X)] 
 \\
 var_{X\sim p_1}[\bar{f}] = \frac{1}{n}var_{X \sim p_1}[f(X)]
$$
因此，$\bar{f}$ 是对于$E_{X\sim p_1}=E_{X\sim p_0}[X]$ 很好的一个估计，即下式。其中$\frac{p_0(x_i)}{p_1(x_i)}$ 被称作重要性权重。
重要性权重大于1能够突出此样本的重要性。
$$
E_{X\sim p_0}[X] \approx \frac{1}{n}\sum_{i=1}^nf(x_i) = \frac{1}{n}\sum_{i=1}^n\frac{p_0(x_i)}{p_1(x_i)}x_i
$$
重要性采样适用于，已知x后很容易求得p（x），但很难求得期望E。例如深度学习的黑盒问题，不知道p的表达式，只能给定x求得p（x）的值。

#### off-policy actor-critic 算法

假设$\beta$ 为off-policy 中behavior policy的经验采样，目的为应用这个采样来更新target policy以优化目标函数$J(\theta) = \sum_{s\in S}d_\beta(s)v_\pi(s) = E_{S \sim d_\beta}[v_\pi(S)]$ 其中 $d_\beta$是策略$\beta$的稳态分布。

此时目标函数经重要性采样优化后格式为
$$
\triangledown_\theta J(\theta) = E_{S \sim p , A \sim \beta}\Big[ \frac{\overbrace{\pi(A|S,\theta)}^{P_0}}{\underbrace{\beta(A|S)}_{P_1}}\triangledown_\theta ln\pi(A|S,\theta)q_\pi(S,A)  \Big]
$$
off-policy 的策略梯度同样也与基准偏差b(s)不相关，所以为了优化方差，通常也会加入基准偏差。（次优b(s)=$v_\pi(s)$)
$$
\triangledown_\theta J(\theta) = E_{S \sim p , A \sim \beta}\Big[ \frac{\overbrace{\pi(A|S,\theta)}^{P_0}}{\underbrace{\beta(A|S)}_{P_1}}\triangledown_\theta ln\pi(A|S,\theta)(q_\pi(S,A)-v_\pi(s) ) \Big]
$$
与之对应的梯度上升算法为
$$
\theta_{t+1} = \theta_t + \alpha_\theta\frac{\pi(a_t|s_t,\theta_t)}{\beta(a_t|s_t)}\triangledown_\theta ln\pi(a_t|s_t,\theta_t)(q_t(s_t,a_t)-v_t(s_t))
$$
同on-policy中情况，利用TD error近似
$$
q_t(s_t,a_t)-v_t(s_t) \approx r_{t+1} + \gamma v_t(s_{t+1}) - v_t(s_t) = \delta_t(s_t,a_t)
\\
且
\\\triangledown_\theta ln\pi(a_t|s_t,\theta_t) = \frac{\triangledown_\theta \pi(a_t|s_t,\theta_t)}{\pi(a_t|s_t,\theta_t)}
$$
带入式中得到最终形式为
$$
\theta_{t+1} = \theta_t + \alpha_\theta\frac{\delta_t(s_t,a_t)}{\beta(a_t|s_t)}\triangledown_\theta \pi(a_t|s_t,\theta_t)
$$

#### 伪代码

初始化：已知给定的behavior policy $\beta(a|s)$ , target policy $\pi(a|s,\theta_0)$ 中$θ_0$ 为初始参数向量，值函数$v(s,w_0)$其中$w_0$是初始参数向量
目标：寻找能优化目标函数的最优策略。
At the step t in each episode ,do 
		遵循behavior policy生成$a_t$，并观察对应的$r_{t+1},s_{t+1}$
		TD error (advantage function)
				$\delta_t = r_{t+1} + \gamma v_t(s_{t+1},w_t) - v_t(s_t,w_t)$
		Critic (value update)
				$w_{t+1} = w_t + \alpha_\theta\frac{\pi(a_t|s_t,\theta_t)}{\beta(a_t|s_t)} \delta_t \triangle_w v(s_t,w_t)$ 
		Actor(policy update)
				$\theta_{t+1} = \theta_t + \alpha_\theta\frac{\pi(a_t|s_t,\theta_t)}{\beta(a_t|s_t)} \delta_t \triangledown_\theta ln\pi(a_t|s_t,\theta_t)$ 



## 4. Deterministic actor-critic(DPG)

deterministic policy相比于 stochastic policy的区别在于对于s与a的映射。其中，$\mu$是从s到a的映射，被简写成$\mu(s)$ 
$$
a = \mu (s,\theta) = \mu(s)
$$
考虑在折扣情况下的目标函数,其中$d_0$ 为概率分布，$\sum_{s \in S}d_0(s) = 1$ 
$$
J(\theta) = E[v_\mu(s)] = \sum_{s \in S}d_0(s)v_\mu(s)
$$
当$d_0$ 的选择与$\mu$ 无关时，梯度很容易就能计算得到。**但考虑两个特殊情况**

1.当每次起始状态都为同一特定状态时，此时$d_0(s_0)= 1 \quad and \quad d_0(s \neq s_0) = 0$
2.$d_0$为behavior policy的一种稳态分布并不同于$\mu$ 

### theorem of deterministic policy gradient

$$
\begin{split}
\triangledown_\theta J(\theta) &= \sum_{s \in S}\rho_\mu(s)\triangledown_\theta\mu(s)(\triangledown_aq_\mu(s,a))|_{a=\mu(s)}
\\
&= E_{S \sim \rho_\mu}\big[ \triangledown_\theta\mu(S)(\triangledown_aq_\mu(S,a))|_{a = \mu(S)}   \big]
\end{split}
\\
先对a求梯度，在将所有a用\mu(S)替换，梯度中再无a出现。
$$

因此，确定性梯度下降属于off-policy方法。

### algorithm of deterministic  actor-critic

基于策略梯度，用于优化目标函数的梯度上升算法为
$$
\theta_{t+1} = \theta_t + \alpha_\theta E_{S \sim \rho_\mu}[\triangledown_\theta \mu(S)(\triangledown_aq_\mu(S,a))|_{a=\mu(S)}]
$$
与之对应的随机梯度上升算法为
$$
\theta_{t+1} = \theta_t + \alpha_\theta\triangledown_\theta \mu(s_t)(\triangledown_aq_\mu(s_t,a))|_{a=\mu(s_t)}
$$

#### 伪代码

初始化：已知给定的behavior policy $\beta(a|s)$ , target policy $\mu(s,\theta_0)$ 中$θ_0$ 为初始参数向量，值函数$v(s,w_0)$其中$w_0$是初始参数向量
目标：寻找能优化目标函数的最优策略。
At time step t in each episode ,do 
		遵循behavior policy生成$a_t$，并观察对应的$r_{t+1},s_{t+1}$
		TD error (advantage function)
				$\delta_t = r_{t+1} + \gamma q(s_{t+1},\mu(s_{t+1,\theta_t},w_t) - q(s_t,a_t,w_t))$
		Critic (value update)
				$w_{t+1} = w_t + \alpha_w \delta_t \triangledown_w q(s_t,a_t,w_t)$ 
		Actor(policy update)
				$\theta_{t+1} = \theta_t + \alpha_\theta \triangledown_\theta\mu(s_t,\theta_t)(\triangledown_aq(s_t,a,w_{t+1}))|_{a = \mu(s_t)}$ 

#### 备注

如何选取函数来表示q(s,a,w)
**Linear function**: q(s,a,w) = $\phi^T(s,a)w$ 其中$\phi(s,a)$为特征向量
**Neural networks:** 深度确定性策略梯度方法（DDPG）