

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

### Baseline invariance

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

#### 重要性采样

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
![image-20250915135028681](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915135028681.png)

![image-20250915135228232](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915135228232.png)



![image-20250915135619145](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915135619145.png)

![image-20250915135715280](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915135715280.png)

![image-20250915135746698](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915135746698.png)

![image-20250915164758826](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915164758826.png)

![image-20250915164945587](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250915164945587.png)



## 4. Deterministic actor-critic(DPG)