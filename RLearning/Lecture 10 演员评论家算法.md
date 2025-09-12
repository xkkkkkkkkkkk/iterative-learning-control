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


![image-20250912162847758](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912162847758.png)

![image-20250912163035933](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912163035933.png)

![image-20250912163216155](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912163216155.png)

### The algorithm of advantage actor-critic

![image-20250912163456134](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912163456134.png)

![image-20250912163707898](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912163707898.png)

![image-20250912163845586](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912163845586.png)

![image-20250912163908530](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250912163908530.png)



## 3. Off-policy actor-critic

## 4. Deterministic actor-critic(DPG)