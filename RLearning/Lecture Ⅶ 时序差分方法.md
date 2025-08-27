# Lecture  Ⅶ  Temporal  Difference  Learning

## 1. Motivating examples

![image-20250827093316319](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093316319.png)

![image-20250827093713473](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093713473.png)

![image-20250827093823428](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093823428.png)

![image-20250827093931845](D:\Users\crcrisoft\AppData\Roaming\Typora\typora-user-images\image-20250827093931845.png)

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

## 4. TD learning of action values: Expected Sarsa

## 5. TD learning of action values : n-step Sarsa

## 6. TD learning of optimal action values : Q-learning

## 7. A unified point of view

## 8. Summary









