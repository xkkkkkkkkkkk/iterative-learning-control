# Lecture Ⅲ    贝尔曼最优公式

#### 两个概念：optimal state value 和 optimal policy

#### 一个工具：贝尔曼最优公式

## 1. Motivating example

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250820110418456.png" alt="image-20250820110418456" style="zoom: 67%;" />

<img src="D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250820110435926.png" alt="image-20250820110435926" style="zoom:80%;" />

可以看出当前的policy不是最好的，s1处的action朝着forbidden区域走

此时就需要依靠action value寻找最优的策略$\pi_{new}$  

在每一个状态都通过对应的action value选取一个最优的策略，随后跳转重复操作。经过这样迭代操作最后一定会选出一个全局最优的策略，这个方法对应的公式就是贝尔曼最优。

## 2 . Definition of optimal policy

 定义：对于一个策略$\pi^*$ 其 state value 比其他任意的策略$\pi $ 所得到的state value都要大，即$ v_{\pi_*}(s)\geq v_\pi(s)$ ,则称此时的策略$\pi^*$ 是最优的

## 3. BOE: Introduction

贝尔曼最优公式（Bellman optimal equation）
$$
v(s) = \underset{\pi}{max} \sum_{a}{\pi(a|s)}\Big(\sum_{r}{p(r|s,a)r+ \gamma \sum_s'{{p(s'|s,a)v(s')}}} \Big)  \\=\underset{\pi}{max} \sum_{a}{\pi(a|s)}q(s,a)   \\ \forall s \in S
$$
在贝尔曼公式中，除了state value v(s)其余所有量都是已知的，而在贝尔曼最优公式中，策略$\pi$ 是不知道的，需要先求出最优策略随后才能求对应的state value

BOE矩阵形式：$ v = \underset{\pi}{max}(r_\pi+\gamma P_\pi v)$

## 4. BOE: Maximization on the right-hand side

贝尔曼最优公式是一个等式求解两个变量的问题，首先求解最优策略$\pi$ 随后在求解state value

#### 此时只需要先求解策略 $\pi$ 的最大值问题

方法实例，现有公式$ r = c_1q_1+c_2q_2+ c_3q_3$ 且$c_1+c_2+c_3 = 1\quad q_1,q_2,q_3$ 中存在最大值$q_*$ ，求r = r_max时，c1-c3对应值。易知，当$q_*$对应的$c_*$=1时，r = r_max。

同理，贝尔曼最优公式中的q(s,a)都是已知量，策略$\pi$即为对应action value最大的策略，至此策略最大值问题解决。随后的state value求解问题同贝尔曼公式求解，将各个state的贝尔曼公式联立随后迭代求解。

## 5. BOE: Rewrite as v = f(v)

将贝尔曼最优公式进行改写$ v = \underset{\pi}{max}(r_\pi+\gamma P_\pi v)$ 

令$\underset{\pi}{max}(r_\pi+\gamma P_\pi v)$  = f(v)，则原式变为v = f(v)

其中$ \underbrace{[f(v)]_s}_{f(v)矩阵形式中的第s个元素} = \underset{\pi}{max} \sum_{a}\pi(a|s)q(s,a)\quad s\in S$ 

随后，求解贝尔曼最优公式，只需求解上述公式即可。

## 6. Contraction mapping theorem

#### Fixed point不动点：

$x \in X$ 并且存在映射f(x)使得f(x) = x，点x被称为不动点

#### Contraction mapping（contractive function）收缩映射：

一个函数f满足以下条件
$$
\lvert f(x_1) - f(x_2)\rvert \leq \gamma \lvert x_1 - x_2\rvert \\ 其中 \gamma \in (0,1)\\ 此处的范数可以为任意范数
$$

#### 理论：Contraction Mapping Theorem 

若函数f是一个contraction mapping，则对于等式$x = f(x)$ 存在以下性质：

①存在性：一定存在解$x_*$ 满足$f(x_*) = x_*$ 

②唯一性：此时的不动点$x_*$是唯一的。

③算法：使用迭代序列{$x_k$}，其中$x_{k+1} = f(x_k)$ ,则当k$\rightarrow \infin$时    $ x_k \rightarrow x_*$ 

## 7. BOE: Solution

为了应用contraction mapping theorem求解贝尔曼最优公式，首先需要证明f(v)是一个contraction mapping。
$$
\vert f(v_1)-f(v_2)\vert \leq \gamma \vert v_1 - v_2 \vert
$$
式中，γ为折扣率，满足contraction mapping条件。故可以用contraction mapping theorem求解f(v)

使用迭代序列{$x_k$}，其中$v_{k+1} = f(v_k)$ ,则当k$\rightarrow \infin$时    $ v_k \rightarrow v_*$ 

#### ==注意==，此时得到的state value是唯一的最优的，但对应的policy不一定是唯一的。

#### 个人理解：

求解贝尔曼公式时，针对一个等式求解两个变量的问题，先解决最大policy问题，此时将v看作常量。随后求解出最大的策略$\pi_*$ 后，等式就变成了求解v的等式$v = f(v)$ ,此时再用contraction mapping theorem求解出最优$v_{\pi_*}(s)$.

## 8. BOE: Optimality

![image-20250820163839144](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250820163839144.png)对于贝尔曼最优公式的解$v^*$ ,他的state value 是所有v中最大的state value，与之对应的$ \pi^*$ 也一定是最优策略。反过来说，其他任何的policy对应得到的state value都没有$ \pi^*$得到的大。

## 9. Analyzing optimal policies

### 影响最优策略的因素

在贝尔曼最优公式中以下三点决定了最优策略
$$
v(s) = \underset{\pi}{max} \sum_{a}{\pi(a|s)}\Big(\sum_{r} \underbrace {{p(r|s,a)}}_{①} \underbrace {r}_{②}+ \underbrace{\gamma}_{③} \sum_s'{{p(s'|s,a)v(s')}} \Big)
$$
①：系统模型，一般不做调整

②：回报r，调高惩罚的reward会让agent变得更保守（犯错成本更高 ）

③：折扣率γ，γ较大agent会比较远视，γ较小会注重短期收益

其余量都是未知且需要求出的量。

### 最优策略的不变性

值得注意的是，当$r\rightarrow ar+b$ ,即所有的reward加上相同的偏执量后，系统的optimal policy不会发生改变。影响optimal policy的是是各reward之间的==relative value==

