# Lecture Ⅳ 值迭代与策略迭代

## 1. Value iteration algorithm

在贝尔曼最优公式中出现的contraction mapping theorem对应的迭代公式$ v_{k+1} = \underset{\pi}{max}(r_\pi+\gamma P_\pi v_k)$ 就被叫做值迭代==value iteration==

值迭代算法可以分为两步

#### ①Policy update(PE)，即优先处理最大policy 的问题，此时$v_k$ 是给定的

$$
\pi_{k+1} = arg\underset{\pi}{max}(r_\pi + \gamma P_\pi v_k)
$$
$\pi_{k+1}(s) = arg\ \underset{\pi}{max} \sum_{a}{\pi(a|s)}\Big(\sum_{r}{p(r|s,a)r+ \gamma \sum_s'{{p(s'|s,a)v_k(s')}}} \Big)  $

此时最优策略的解为$\pi_{k+1}(a|s) = \begin{cases} 1\quad a = a_k^*(s)\\ 0 \quad a ≠ a_k^*(s)  \end{cases}$ 

其中$ q_k^*(s)  = arg {max_a}q_k(a,s)$ $\pi_{k+1}$ 被称为贪婪策略

#### ②Value upddate(VU)，求解出最大策略后便可联立求解state value

$$
v_{k+1} = r_{\pi_{k+1}} + \gamma P_{\pi_{k+1}}v_k 
$$
==注意==，上式右侧的$v_k$ 并不是一个真正的state value，只是一个值（其作为state value迭代求解的一个过程量存在）

#### 伪代码步骤：

while  $\vert v_k - v_{k-1} \vert$ < 阈值没有实现

​		For every state s $\in$ S,do

​				For every action a$\in$ A .do

​						求解$q_k(s,a)

​				得到最大的$a_k^*(s)$  = $arg max_a q_k(a,s)$ 

​				策略更新：$\pi_{k+1}(a,s) = 1 if a = a_k^* and \pi_{k+1}(a,s) =0 $   otherwise 

​				值更新： $ v_{k+1}(s) = max_a q_k(a,s)$  

## 2. Policy iteration algorithm

### 算法描述：

给定一个随机的初始策略$\pi_0$ 

#### 步骤1：policy evaluation（PE）

根据给定的策略 $\pi_k$ ，求出对应的state value 
$$
v_{\pi k} = r_{\pi k } + \gamma P{\pi k} v_{\pi k }
$$

#### 步骤2：policy improvement（PI）

根据求得的state value，求解优化问题得出新的策略$\pi_{k+1}$
$$
\pi_{k+1} = arg \underset{\pi}{max}(r_\pi + \gamma P_\pi v_{\pi k })
$$
$$
\pi_0 \xrightarrow{PE} v_{\pi 1} \xrightarrow{PI} \pi_1 \xrightarrow{PE} v_{\pi 2} \xrightarrow{PI} \pi_2\ldots
$$

#### 一.Policy evaluation

同贝尔曼公式的求解一样有两种方法，直接解法与迭代求解。一般使用迭代求解进行编程。公式如下
$$
\large{v_{\pi_k}^{\overbrace{(j+1)}^{{policy\ evaluation求解中的迭代次数}}} = r_{\pi_k} + \gamma P_{\pi_k}v_{\pi_k}^{(j)}}
$$

#### ==注意==Policy iteration本来就是一个循环，其内部的求解过程policy evaluation也是一个循环，属于嵌套结构。

#### 二.新旧策略对比

通俗来讲，选择策略时是根据每个policy对应的state value进行迭代的（greedy policy），也就是说每次迭代中state value最大的那个策略才会被选为新策略。此处有一引理证明新策略的state value一定比旧策略的大，即
$$
If \ \pi_{k+1} = arg\underset{\pi}{max}(r_\pi + \gamma P_\pi v_{\pi_k})\quad,then\quad v_{\pi_{k+1}} > v_{\pi_k}\quad for\ any\ k
$$
![image-20250821143427445](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250821143427445.png)

#### 三.最优策略的存在性

通过PI和PE两个步骤可以易知，$v_{\pi 0} \leq v_{\pi 1} \leq v_{\pi 2}\leq v_{\pi 3}\ldots \leq v^*$ 由此得出结论$ v_{\pi k}$ 会一直上升直至收敛。得到的optimal state value对应的policy也就是optimal policy了。

### 具体实现：

While 策略未收敛，for the $k$th iteration ,do 

​		Policy evaluation :

​		初始化：初始预设policy $v_{\pi_k}^{(0)}$ 

​		While  $v_{\pi_k}^{(j))}$ 尚未收敛，for the $j$th iteration , do 

​				For every state s $\in$ S ,do 

​						$v_{\pi_k}^{(j+1)}(s) = \sum_{a}{\pi_k(a,s)} \big[ \sum_{r}{p(r|s,a)r + \gamma \sum_{s'}{p(s'|s,a)v_{\pi_k}^{(j)}(s') }} \big]$							

​		Policy improvement:

​		For every s $\in$ S ,do 

​				For every action a $\in$ A(s), do 

​						$q_{\pi_k}(s,a) = \sum_{r}{p(r|s,a)r}+ \gamma \sum_{s'}{p(s'|s,a)v_{\pi_k}(s')}	$			

​				$a_k^*(s) = arg\ \underset{a}{max} q_{\pi_k}(s,a) $

​				$\pi_{k+1}(a|s) = 1 \quad if a = a_k^*,\quad \pi_{k+1}(a|s) = 0 \ otherwise$

## 3. Truncatd policy iteration algorithm

### 比较值迭代和策略迭代的区别

#### 策略迭代： 开始于策略$\pi_0 \rightarrow v_\pi^*$ 

Policy evaluation(PE): $v_{\pi_k} = r_{\pi_k}+  P_{\pi_k}v_{\pi_k}$ 

Policy improvement(PI): $\pi_{k+1} =  arg\ \underset{\pi}{max}(r_\pi+\gamma P_{\pi_k} v_{\pi _k})$ 

#### 值迭代： 开始于$v_0 \rightarrow v^*$

Policy update(PU): $\pi_{k+1} = arg \underset{\pi}{max}(r_\pi + \gamma p_\pi v_k)$ 

Value update(VU): $v_{k+1} = r_{\pi_{k+1}} + \gamma P_{\pi_{l+1}}v_k$  

#### 两个算法非常相似

对于Policy iteration： $\pi_0 \xrightarrow{PE} v_{\pi0} \xrightarrow{PI} \pi_1 \xrightarrow{PE} v_{\pi2} \xrightarrow{PE} v_{\pi2} \xrightarrow{PI}\ldots$ 

对于Value iteration：$\qquad\qquad u_{0} \xrightarrow{PI} \pi_1' \xrightarrow{PE} u_{2} \xrightarrow{PE} u_{2} \xrightarrow{PI}\ldots$ (u0不是v0，因为是通过前一步的v得出的估计值，不是准确量)

​    ![image-20250821163141945](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250821163141945.png)

#### 两者从同一初始条件出发，前三步相同，第四步开始出现差异

#### policy iteration中，求解$v_{\pi_1} = r_{\pi_1}+ \gamma P_{\pi_1}v_{\pi_1}$ 需要迭代算法（无穷次迭代）

#### value iteration中，求解$v_1 = r_{\pi_1}+ \gamma P_{\pi_1}v_0$ 是一个one-step迭代

![image-20250821163925958](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250821163925958.png)

#### 对于求解state value这一步，value iteration只进行一次计算；policy iteration进行无穷次迭代的计算；truncated policy iteration进行 j 次迭代的计算

### 阶段策略迭代

#### 伪代码部分，与policy iteration基本相同，唯一差别在于循环条件从对于$v_k$ 收敛变成了固定迭代次数

![image-20250821160732345](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250821160732345.png)

#### truncated policy iteration收敛性

![image-20250821160716005](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250821160716005.png)

## Summary

#### Value iteration：用于求解贝尔曼最优公式的迭代算法，给定的初始值是value $v_0$。分为PU和VU两部分

#### Policy iteration：给定的初始值是policy $\pi_0$ 。分为PI和PE两部分

#### Truncated policy iteration：在求解$v_\pi$上迭代有限次的policy iteration。VI 和PI都是其极限情况。

