本文为读者提供一种SO(3)上的ESKF（Error state Kalman Filter，误差卡尔曼滤波器）（有时也叫流形上的ESKF）推导过程。整个过程比大多数材料要简明一些，没有太多奇怪的符号，都是大家平日里常用、熟悉的内容，思路上也平铺直叙，没有很多需要跳跃的地方。保证一看就懂，一推就会，供各位参考。如果读者发现错误，还望及时指正。

## ESKF原理

在现代的大多数IMU系统中，人们往往使用误差状态卡尔曼滤波器（Error state Kalman filter, ESKF）而非原始状态的卡尔曼滤波器。大部分基于滤波器的LIO或VIO实现中，都使用ESKF作为状态估计方法[1,2]。相比于传统KF，ESKF的优点可以总结如下[3]：

1. 在旋转的处理上，ESKF的状态变量可以采用最小化的参数表达，也就是使用三维变量来表达旋转的增量。而传统KF需要用到四元数（4维）或者更高维的表达（旋转矩阵，9维），要不就得采用带有奇异性的表达方式（欧拉角）。
2. ESKF总是在原点附近，离奇异点较远，并且也不会由于离工作点太远而导致线性化近似不够的问题。
3. ESKF的状态量为小量，其二阶变量相对来说可以忽略。同时大多数雅可比矩阵在小量情况下变得非常简单，甚至可以用单位阵代替。
4. 误差状态的运动学也相比原状态变量要来得更小，因为我们可以把大量更新部分放到原状态变量中。

在ESKF中，我们通常把原状态变量称为**名义状态变量**（nominal state），然后把ESKF里的状态变量称为**误差状态变量**（error state）。ESKF整体流程如下：当IMU测量数据到达时，我们把它积分后，放入名义状态变量中。由于这种做法没有考虑噪声，其结果自然会快速漂移，于是我们希望把误差部分作为误差变量，放在ESKF中。ESKF内部会考虑各种噪声和零偏的影响，并且给出误差状态的一个高斯分布描述。同时，ESKF本身作为一种卡尔曼滤波器，也具有预测过程和修正过程，其中修正过程需要依赖IMU以外的传感器观测。当然，在修正之后，ESKF可以给出后验的误差高斯分布，随后我们可以把这部分误差放入名义状态变量中，并把ESKF置零，这样就完成了一次循环[4]。

------

### ESKF状态方程

3我们设ESKF的真值状态为： ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bx%7D_t+%3D+%5B%5Cbm%7Bp%7D_t%2C+%5Cbm%7Bv%7D_t%2C+%5Cbm%7BR%7D_t%2C+%5Cbm%7Bb%7D_%7Bat%7D%2C+%5Cbm%7Bb%7D_%7Bgt%7D%2C+%5Cbm%7Bg%7D_t%5D%5E%5Cmathrm%7BT%7D) 。这个状态随时间改变，可以记 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bx%7D%28t%29_t) 。在连续时间上，我们记IMU读数为 ![[公式]](https://www.zhihu.com/equation?tex=%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D%2C+%5Ctilde%7B%5Cbm%7Ba%7D%7D) ，那么可以写出状态变量导数相对于观测量之间的关系式：

![[公式]](https://www.zhihu.com/equation?tex=+%09%5Cbegin%7Balign%7D+%09%09%5Cdot%7B%5Cbm%7Bp%7D%7D_t+%26%3D+%5Cbm%7Bv%7D_t+%5C%5C+%09%09%5Cdot%7B%5Cbm%7Bv%7D%7D_t+%26%3D+%5Cbm%7BR%7D_t+%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_%7Bat%7D+-+%5Cboldsymbol%7B%5Ceta%7D_a%29+%2B+%5Cbm%7Bg%7D+%5C%5C+%09%09%5Cdot%7B%5Cbm%7BR%7D%7D_t+%26%3D+%5Cbm%7BR%7D_t+%5C++%5Cleft%28+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%29%5E%5Cwedge+%5C%5C+%09%09%5Cdot%7B%5Cbm%7Bb%7D%7D_%7Bgt%7D+%26+%3D+%5Cboldsymbol%7B%5Ceta%7D_%7Bbg%7D+%5C%5C+%09%09%5Cdot%7B%5Cbm%7Bb%7D%7D_%7Bat%7D+%26+%3D+%5Cboldsymbol%7B%5Ceta%7D_%7Bba%7D+%5C%5C++%09%09%5Cdot%7B%5Cbm%7Bg%7D%7D+%26%3D+%5Cbm%7B0%7D+%09%5Cend%7Balign%7D+)

其中带下标 ![[公式]](https://www.zhihu.com/equation?tex=t) 的表示真值。相信读者已经熟悉这些公式的含义了。这里把重力 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bg%7D) 考虑进来的主要理由是方便确定IMU的初始姿态。如果我们不在状态方程里写出重力变量，那么必须事先确定初始时刻的IMU朝向 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BR%7D%280%29) ，才可以执行后续的计算。此时IMU的姿态就是相对于初始的水平面来描述的。而如果把重力写出来，就可以设IMU的初始姿态为单位矩阵 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BR%7D%3D%5Cbm%7BI%7D) ，而把重力方向作为IMU当前姿态相比于水平面的一个度量。二种方法都是可行的，不过将重力方向单独表达出来会使得初始姿态表达更加简单，同时还可以增加一些线性性[5]。

如果把观测量和噪声量整理的一个向量，我们也可以把上式整理成矩阵形式。不过这里的矩阵形式将含有很多的零项，相比上式并不会有明显简化，所以我们就先使用这种散开的公式。下面我们来推导误差状态方程。首先定义误差状态变量为：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cbm%7Bp%7D_t+%26%3D+%5Cbm%7Bp%7D+%2B+%5Cdelta+%5Cbm%7Bp%7D+%5C%5C+%09%09%5Cbm%7Bv%7D_t+%26%3D+%5Cbm%7Bv%7D+%2B+%5Cdelta+%5Cbm%7Bv%7D+%5C%5C+%09%09%5Cbm%7BR%7D_t+%26%3D+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7BR%7D+%5Cquad++%5Ctext%7B%E6%88%96%7D+%5C+%5Cbm%7Bq%7D_t+%3D+%5Cbm%7Bq%7D+%5Cdelta+%5Cbm%7Bq%7D+%5C%5C+%09%09%5Cbm%7Bb%7D_%7Bgt%7D+%26%3D+%5Cbm%7Bb%7D_g+%2B+%5Cdelta+%5Cbm%7Bb%7D_g+%5C%5C+%09%09%5Cbm%7Bb%7D_%7Bat%7D+%26%3D+%5Cbm%7Bb%7D_a+%2B+%5Cdelta+%5Cbm%7Bb%7D_a+%5C%5C+%09%09%5Cbm%7Bg%7D_t+%26%3D+%5Cbm%7Bg%7D+%2B+%5Cdelta+%5Cbm%7Bg%7D+%09%5Cend%7Balign%7D)

不带下标的就是**名义状态变量**。**名义状态变量**的运动学方程式与真值相同，只是**不必考虑噪声**（因为噪声在误差状态方程中考虑了）。其中旋转部分的 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7BR%7D) 可以用它的李代数 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29) 来表示，此时旋转公式也需要改成用指数形式来表达。关于误差变量的平移、零偏和重力公式，都很容易得出对应的时间导数表达式，只需在等式两侧分别对时间求导即可：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bp%7D%7D+%26%3D+%5Cdelta+%5Cbm%7Bv%7D+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bb%7D_g%7D+%26%3D+%5Cboldsymbol%7B%5Ceta%7D_g+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bb%7D_a%7D+%26%3D+%5Cboldsymbol%7B%5Ceta%7D_a+%5C%5C+%09%09%5Cdelta+%5Cbm%7Bg%7D+%26%3D+%5Cbm%7B0%7D+%09%5Cend%7Balign%7D)

而速度、旋转两式由于和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7BR%7D) 有关系，所以要单独推导。

------

### 误差状态的旋转项

对旋转式两侧求时间导数，可得：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%09%09%5Cdot%7B%5Cbm%7BR%7D%7D_t+%26%3D+%5Cdot%7B%5Cbm%7BR%7D%7D+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%2B+%5Cbm%7BR%7D+%5Cdot%7B%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29%7D++%5C%5C+%09%09%26%3D++%5Cbm%7BR%7D_t+%5Cleft%28+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%29+%5E%5Cwedge+%09%5Cend%7Baligned%7D)

该式右边的 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdot%7B%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29%7D) 满足：

![[公式]](https://www.zhihu.com/equation?tex=%5Cdot%7B%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29%7D+%3D+%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D%5E%5Cwedge.)

因此第一个式子可写成：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%09%09%5Cdot%7B%5Cbm%7BR%7D%7D+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%2B+%5Cbm%7BR%7D+%5Cdot%7B%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29%7D+%26%3D+%5Cbm%7BR%7D+%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D-%5Cbm%7Bb%7D_g%29%5E%5Cwedge+%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%2B+%5Cbm%7BR%7D+%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+%29+%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D%5E%5Cwedge+%5C%5C+++%09+%5Cend%7Baligned%7D)

而第二个式子可以写成：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%09%09%5Cbm%7BR%7D_t+%5Cleft%28+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%29%5E%5Cwedge+%26%3D+%5Cbm%7BR%7D++%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%28+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%29%5E%5Cwedge+%5C%5C++%5Cend%7Baligned%7D)

比较这两个式子，将 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D) 移到一侧，约掉两侧左边的 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BR%7D) ，整理类似项，不难得到：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Baligned%7D+%09%09%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D%5E%5Cwedge+%26%3D+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%28+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%29%5E%5Cwedge+-+%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D-%5Cbm%7Bb%7D_g%29%5E%5Cwedge+%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cend%7Baligned%7D)

注意到 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29) 本身是一个 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathrm%7BSO%7D%283%29) 矩阵，我们利用 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathrm%7BSO%7D%283%29) 上的伴随性质：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cboldsymbol%7B%5Cphi%7D%5E%5Cwedge+%5Cbm%7BR%7D+%3D+%5Cbm%7BR%7D+%28%5Cbm%7BR%7D%5E%5Cmathrm%7BT%7D+%5Cboldsymbol%7B%5Cphi%7D%29%5E%5Cwedge%2C)

用来交换上面的 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29) ：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Baligned%7D+%09%09%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D%5E%5Cwedge+%26%3D+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%28+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%29%5E%5Cwedge+-+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29++%5Cleft%28+%5Cmathrm%7BExp%7D+%28-%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29++%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D-%5Cbm%7Bb%7D_g%29+%5Cright%29%5E%5Cwedge+%5C%5C+%09%09+%26%3D++%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%5B+%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g%29%5E%5Cwedge+-+%28%5Cmathrm%7BExp%7D+%28-%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29++%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D-%5Cbm%7Bb%7D_g%29+%29%5E%5Cwedge+%5Cright%5D+%5C%5C+%09%09+%26%5Capprox+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%5B+%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_%7Bgt%7D+-+%5Cboldsymbol%7B%5Ceta%7D_g%29%5E%5Cwedge+-+%5Cleft%28%28%5Cbm%7BI%7D+-+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge%29%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D-%5Cbm%7Bb%7D_g+%29%5Cright%29%5E%5Cwedge+%5Cright%5D+%5C%5C+%09%09+%26%3D++%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%5B+%5Cbm%7Bb%7D_g+-+%5Cbm%7Bb%7D_%7Bgt%7D+-%5Cboldsymbol%7B%5Ceta%7D_g+%2B+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge+%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge+%5Cbm%7Bb%7D_%7Bg%7D+%5Cright%5D%5E%5Cwedge+%5C%5C+%09%09+%26%3D+%5Cmathrm%7BExp%7D+%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%5Cleft%5B+%28-%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D%2B%5Cbm%7Bb%7D_g%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cdelta+%5Cbm%7Bb%7D_g+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5Cright%5D%5E%5Cwedge+%09%5Cend%7Baligned%7D)

约掉等式左侧的系数，可得：

![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D+%5Capprox+-%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_g%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cdelta+%5Cbm%7Bb%7D_g+-+%5Cboldsymbol%7B%5Ceta%7D_g)

### 误差状态的速度项

接下来考虑速度方程的误差形式。同样地，对两侧求时间导数，就可以得到 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cdot%7B%5Cbm%7Bv%7D%7D) 的表达式。等式左侧为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D+%09%5Cbegin%7Baligned%7D+%09%09%5Cdot%7B%5Cbm%7Bv%7D%7D_t+%26%3D+%5Cbm%7BR%7D_t%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_%7Bat%7D+-+%5Cboldsymbol%7B%5Ceta%7D_a%29+%2B+%5Cbm%7Bg%7D_t+%5C%5C+%09%09%26%3D+%5Cbm%7BR%7D+%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29+%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a+-+%5Cdelta+%5Cbm%7Bb%7D_a+-+%5Cboldsymbol%7B%5Ceta%7D_a+%29+%2B+%5Cbm%7Bg%7D+%2B+%5Cdelta+%5Cbm%7Bg%7D+%5C%5C+%09%09%26%5Capprox+%5Cbm%7BR%7D+%28%5Cbm%7BI%7D+%2B+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge+%29+%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a+-+%5Cdelta+%5Cbm%7Bb%7D_a+-+%5Cboldsymbol%7B%5Ceta%7D_a%29+%2B+%5Cbm%7Bg%7D+%2B+%5Cdelta+%5Cbm%7Bg%7D+%5C%5C+%09%09%26%5Capprox+%5Cbm%7BR%7D+%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7BR%7D+%5Cbm%7Bb%7D_a+-+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7Bb%7D_a+-+%5Cbm%7BR%7D+%5Cboldsymbol%7B%5Ceta%7D_a+%2B+%5Cbm%7BR%7D+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge+%5Cbm%7Ba%7D+-+%5Cbm%7BR%7D+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge+%5Cbm%7Bb%7D_a+%2B+%5Cbm%7Bg%7D+%2B+%5Cdelta+%5Cbm%7Bg%7D+%5C%5C+%09%09%26%3D+%5Cbm%7BR%7D+%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7BR%7D+%5Cbm%7Bb%7D_a+-+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7Bb%7D_a+-+%5Cbm%7BR%7D+%5Cboldsymbol%7B%5Ceta%7D_a+-+%5Cbm%7BR%7D+%5Ctilde%7B%5Cbm%7Ba%7D%7D%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D++%2B+%5Cbm%7BR%7D+%5Cbm%7Bb%7D_a%5E%5Cwedge++%5Cdelta%5Cboldsymbol%7B%5Ctheta%7D++%2B+%5Cbm%7Bg%7D+%2B+%5Cdelta+%5Cbm%7Bg%7D+%09%5Cend%7Baligned%7D+%5Cend%7Bequation%7D)

从第三行推向第四行时，需要忽略 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%5E%5Cwedge) 与 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7Bb%7D_a%2C+%5Cboldsymbol%7B%5Ceta%7D_a) 相乘的二阶小量。从第四行推第五行则用到了叉乘符号交换顺序之后需加负号的性质。另一方面，等式右侧为：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cdot%7B%5Cbm%7Bv%7D%7D+%2B+%5Cdelta+%5Cdot%7B%5Cbm%7Bv%7D%7D+%3D+%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29+%2B+%5Cbm%7Bg%7D+%2B+%5Cdelta+%5Cdot%7B%5Cbm%7Bv%7D%7D)

因为上面两式相等，可以得到：

![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cdot%7B%5Cbm%7Bv%7D%7D+%3D+-+%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7Bb%7D_a++-+%5Cbm%7BR%7D+%5Cboldsymbol%7B%5Ceta%7D_a+%2B+%5Cdelta+%5Cbm%7Bg%7D)

这样我们就得到了 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7Bv%7D) 的运动学模型。需要补充一句，由于上式中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cboldsymbol%7B%5Ceta%7D_a) 是一个零均值白噪声，它乘上任意旋转矩阵之后仍然是一个零均值白噪声，而且由于 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BR%7D%5E%5Cmathrm%7BT%7D+%5Cbm%7BR%7D+%3D+%5Cbm%7BI%7D) ，其协方差矩阵也不变（留作习题）。所以，也可以把上式简化为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D%5Clabel%7Bkey%7D+%09%5Cdelta+%5Cdot%7B%5Cbm%7Bv%7D%7D+%3D+-+%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7Bb%7D_a++-+%5Cboldsymbol%7B%5Ceta%7D_a+%2B+%5Cdelta+%5Cbm%7Bg%7D+%5Cend%7Bequation%7D)

至此，我们可以把误差变量的运动学方程整理如下：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bp%7D%7D+%26%3D+%5Cdelta+%5Cbm%7Bv%7D+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bv%7D%7D+%26%3D+-+%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7Bb%7D_a++-+%5Cboldsymbol%7B%5Ceta%7D_a+%2B+%5Cdelta+%5Cbm%7Bg%7D+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cboldsymbol%7B%5Ctheta%7D%7D+%26%3D+-%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_g%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cdelta+%5Cbm%7Bb%7D_g+-+%5Cboldsymbol%7B%5Ceta%7D_g+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bb%7D_g%7D+%26%3D+%5Cboldsymbol%7B%5Ceta%7D_%7Bbg%7D+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bb%7D_a%7D+%26%3D+%5Cboldsymbol%7B%5Ceta%7D_%7Bba%7D+%5C%5C+%09%09%5Cdelta+%5Cdot%7B%5Cbm%7Bg%7D%7D+%26%3D+%5Cbm%7B0%7D++%09%5Cend%7Balign%7D)

------

### 离散时间的ESKF运动学方程

从连续时间状态方程推出离散时间的状态方程并不困难，不妨直接来列写它们。名义状态变量的离散时间运动学方程可以写为：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cbm%7Bp%7D%28t%2B%5CDelta+t%29+%26%3D+%5Cbm%7Bp%7D%28t%29+%2B+%5Cbm%7Bv%7D+%5CDelta+t+%2B+%5Cfrac%7B1%7D%7B2%7D+%5Cleft%28%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D-%5Cbm%7Bb%7D_a%29+%5Cright%29+%5CDelta+t%5E2+%2B+%5Cfrac%7B1%7D%7B2%7D+%5Cbm%7Bg%7D+%5CDelta+t%5E2%5C%5C+%09%09%5Cbm%7Bv%7D%28t%2B%5CDelta+t%29+%26%3D+%5Cbm%7Bv%7D%28t%29+%2B+%5Cbm%7BR%7D+%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29+%5CDelta+t+%2B+%5Cbm%7Bg%7D+%5CDelta+t%09%5C%5C+%09%09%5Cbm%7BR%7D%28t%2B%5CDelta+t%29+%26%3D+%5Cbm%7BR%7D%28t%29+%5Cmathrm%7BExp%7D+%5Cleft%28+%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D-%5Cbm%7Bb%7D_g%29+%5CDelta+t+%5Cright%29%5C%5C+%09%09%5Cbm%7Bb%7D_g%28t%2B%5CDelta+t%29+%26%3D+%5Cbm%7Bb%7D_g%28t%29+%5C%5C+%09%09%5Cbm%7Bb%7D_a%28t%2B%5CDelta+t%29+%26%3D+%5Cbm%7Bb%7D_a%28t%29+%5C%5C+%09%09%5Cbm%7Bg%7D%28t%2B%5CDelta+t%29+%26%3D+%5Cbm%7Bg%7D%28t%29++%09%5Cend%7Balign%7D)

该式只需在上面的基础上添加零偏项与重力项即可。而误差状态的离散形式则只需要处理连续形式中的旋转部分。参考角速度的积分公式，可以将误差状态方程写为：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cdelta+%5Cbm%7Bp%7D%28t%2B%5CDelta+t%29+%26%3D+%5Cdelta+%5Cbm%7Bp%7D+%2B+%5Cdelta+%5Cbm%7Bv%7D+%5CDelta+t+%5C%5C+%09%09%5Cdelta+%5Cbm%7Bv%7D%28t%2B%5CDelta+t%29+%26%3D+%5Cdelta+%5Cbm%7Bv%7D+%2B+%5Cleft%28+-+%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29%5E%5Cwedge+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cbm%7BR%7D+%5Cdelta+%5Cbm%7Bb%7D_a++%2B+%5Cdelta+%5Cbm%7Bg%7D+%5Cright%29+%5CDelta+t+%2B+%5Cboldsymbol%7B%5Ceta%7D_%7Bv%7D+%5C%5C+%09%09%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+%28t%2B%5CDelta+t%29+%26%3D+%5Cmathrm%7BExp%7D%5Cleft%28+-%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_g%29+%5CDelta+t+%5Cright%29+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D+-+%5Cdelta+%5Cbm%7Bb%7D_g+%5CDelta+t+-+%5Cboldsymbol%7B%5Ceta%7D_%7B%5Ctheta%7D+%5C%5C+%09%09%5Cdelta+%5Cbm%7Bb%7D_g+%28t%2B%5CDelta+t%29+%26%3D+%5Cdelta+%5Cbm%7Bb%7D_g+%2B+%5Cboldsymbol%7B%5Ceta%7D_g+%5C%5C+%09%09%5Cdelta+%5Cbm%7Bb%7D_a+%28t%2B%5CDelta+t%29%26%3D+%5Cdelta+%5Cbm%7Bb%7D_a+%2B+%5Cboldsymbol%7B%5Ceta%7D_a+%5C%5C+%09%09%5Cdelta+%5Cbm%7Bg%7D+%28t%2B%5CDelta+t%29+%26%3D+%5Cdelta+%5Cbm%7Bg%7D+%09%5Cend%7Balign%7D)

注意：

1. 右侧部分我们省略了括号里的 ![[公式]](https://www.zhihu.com/equation?tex=%28t%29) 以简化公式；
2. 关于旋转部分的积分，我们可以将连续形式看成关于 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D) 的微分方程然后求解。求解过程类似于对角速度进行积分。
3. 噪声项并不参与递推，需要把它们单独归入噪声部分中。连续时间的噪声项可以视为随机过程的能量谱密度，而离散时间下的噪声变量就是我们日常看到的随机变量了。这些噪声随机变量的标准差可以列写如下： ![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D%5Clabel%7Bkey%7D+%09%09%5Csigma%28%5Cboldsymbol%7B%5Ceta%7D_v%29+%3D+%5CDelta+t+%5Csigma_a%2C+%5Cquad+%5Csigma%28%5Cboldsymbol%7B%5Ceta%7D_%7B%5Ctheta%7D%29+%3D+%5CDelta+t+%5Csigma_%7Bg%7D%2C+%5Cquad+%5Csigma%28%5Cboldsymbol%7B%5Ceta%7D_g%29+%3D+%5Csqrt%7B%5CDelta+t%7D+%5Csigma_%7Bbg%7D%2C+%5Cquad++%5Csigma%28%5Cboldsymbol%7B%5Ceta%7D_a%29+%3D+%5Csqrt%7B%5CDelta+t%7D+%5Csigma_%7Bba%7D+%09%5Cend%7Bequation%7D)

其中前两式的 ![[公式]](https://www.zhihu.com/equation?tex=%5CDelta+t) 是由积分关系导致的。

至此，我们给出了如何在ESKF中进行IMU递推的过程，对应于卡尔曼滤波器中的状态方程。为了让滤波器收敛，我们通常需要外部的观测来对卡尔曼滤波器进行修正，也就是所谓的组合导航。当然，组合导航的方法有很多，从传统的EKF，到本节介绍的ESKF，以及后续章节将要介绍预积分和图优化技术，都可以应用于组合导航中。

### ESKF的运动过程

根据上述讨论，我们可以写出ESKF的运动过程。误差状态变量 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7Bx%7D) 的离散时间运动方程已经在上式给出，我们可以整体地记为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7Bx%7D+%3D+f%28%5Cdelta+%5Cbm%7Bx%7D%29+%2B+%5Cbm%7Bw%7D%2C+%5Cbm%7Bw%7D+%5Csim+%5Cmathcal%7BN%7D%280%2C+%5Cbm%7BQ%7D%29)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bw%7D) 为噪声。按照前面的定义， ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BQ%7D) 应该为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BQ%7D+%3D+%5Cmathrm%7Bdiag%7D%28%5Cbm%7B0%7D_3%2C+%5Cmathrm%7BCov%7D%28%5Cboldsymbol%7B%5Ceta%7D_v%29%2C+%5Cmathrm%7BCov%7D%28%5Cboldsymbol%7B%5Ceta%7D_%7B%5Ctheta%7D%29%2C+%5Cmathrm%7BCov%7D%28%5Cboldsymbol%7B%5Ceta%7D_%7Bg%7D%29%2C+%5Cmathrm%7BCov%7D%28%5Cboldsymbol%7B%5Ceta%7D_%7Ba%7D%29%2C+%5Cbm%7B0%7D_3%29)

两侧的零是由于第一个和最后一个方程本身没有噪声导致的。

为了保持与EKF的符号统一，我们计算运动方程的线性化形式：

![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7Bx%7D+%3D+%5Cbm%7BF%7D+%5Cdelta+%5Cbm%7Bx%7D+%2B+%5Cbm%7Bw%7D)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BF%7D) 为线性化后的雅可比矩阵。由于我们列写的运动方程已经是线性化的了，只需把它们的线性系统拿出来即可：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbm%7BF%7D+%3D+%5Cbegin%7Bbmatrix%7D+%09%09%5Cbm%7BI%7D+%26+%5Cbm%7BI%7D+%5CDelta+t+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%5C%5C+%09%09%5Cbm%7B0%7D+%26+%5Cbm%7BI%7D+%26+-+%5Cbm%7BR%7D%28%5Ctilde%7B%5Cbm%7Ba%7D%7D+-+%5Cbm%7Bb%7D_a%29%5E%5Cwedge+%5CDelta+t+%26+-%5Cbm%7BR%7D+%5CDelta+t+%26+%5Cbm%7B0%7D+%26+%5Cbm%7BI%7D+%5CDelta+t+%5C%5C+%09%09%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cmathrm%7BExp%7D%5Cleft%28+-%28%5Ctilde%7B%5Cboldsymbol%7B%5Comega%7D%7D+-+%5Cbm%7Bb%7D_g%29+%5CDelta+t+%5Cright%29+%26+%5Cbm%7B0%7D+%26+-%5Cbm%7BI%7D+%5CDelta+t+%26+%5Cbm%7B0%7D+%5C%5C+%09%09%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7BI%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%5C%5C+%09%09%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7BI%7D+%26+%5Cbm%7B0%7D+%5C%5C+%09%09%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7B0%7D+%26+%5Cbm%7BI%7D++%09%5Cend%7Bbmatrix%7D)

在此基础上，我们执行ESKF的预测过程。预测过程包括对名义状态的预测（IMU积分）以及对误差状态的预测：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Balign%7D+%09%09%5Cdelta+%5Cbm%7Bx%7D_%7B%5Cmathrm%7Bpred%7D%7D+%26%3D+%5Cbm%7BF%7D+%5Cdelta+%5Cbm%7Bx%7D+%5C%5C+%09%09%5Cbm%7BP%7D_%7B%5Cmathrm%7Bpred%7D%7D+%26%3D+%5Cbm%7BF%7D+%5Cbm%7BP%7D+%5Cbm%7BF%7D%5E%5Cmathrm%7BT%7D+%2B+%5Cbm%7BQ%7D+%09%5Cend%7Balign%7D)

不过由于ESKF的误差状态在每次更新以后会被重置，因此运动方程的均值部分没有太大意义，而方差部分则可以指导整个误差估计的分布情况。

### ESKF的更新过程

前面介绍的是ESKF的运动过程，现在我们来考虑更新过程。假设一个抽象的传感器能够对状态变量产生观测，其观测方程为抽象的 ![[公式]](https://www.zhihu.com/equation?tex=h) ，那么可以写为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bz%7D+%3D+h%28%5Cbm%7Bx%7D%29+%2B+%5Cbm%7Bv%7D%2C+%5Cbm%7Bv%7D+%5Csim+%5Cmathcal%7BN%7D%280%2C+%5Cbm%7BV%7D%29+)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bz%7D) 为观测数据， ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bv%7D) 为观测噪声， ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BV%7D) 为该噪声的协方差矩阵。由于状态变量里已经有 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BR%7D) 了，这里我们换个符号。

在传统EKF中，我们可以直观对观测方程线性化，求出观测方程相对于状态变量的雅可比矩阵，进而更新卡尔曼滤波器。而在ESKF中，我们当前拥有名义状态 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bx%7D) 的估计以及误差状态 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cbm%7Bx%7D) 的估计，且希望更新的是误差状态，因此要计算观测方程相比于误差状态的雅可比矩阵：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BH%7D+%3D+%5Cfrac%7B%5Cpartial+h%7D%7B%5Cpartial+%5Cdelta+%5Cbm%7Bx%7D%7D%2C)

然后再计算卡尔曼增益，进而计算误差状态的更新过程：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cbm%7BK%7D+%26%3D+%5Cbm%7BP%7D_%7B%5Cmathrm%7Bpred%7D%7D+%5Cbm%7BH%7D%5E%5Cmathrm%7BT%7D%28%5Cbm%7BH%7D+%5Cbm%7BP%7D_%7B%5Cmathrm%7Bpred%7D%7D+%5Cbm%7BH%7D%5E%5Cmathrm%7BT%7D+%2B+%5Cbm%7BV%7D%29%5E%7B-1%7D+%5C%5C+%09%09%5Cdelta+%5Cbm%7Bx%7D+%26%3D+%5Cbm%7BK%7D+%28%5Cbm%7Bz%7D+-+h%28%5Cbm%7Bx%7D_t%29%29+%5C%5C+%09%09%5Cbm%7BP%7D+%26%3D+%28%5Cbm%7BI%7D+-+%5Cbm%7BK%7D+%5Cbm%7BH%7D%29+%5Cbm%7BP%7D_%7B%5Cmathrm%7Bpred%7D%7D+%09%5Cend%7Balign%7D)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BK%7D) 为卡尔曼增益， ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BP%7D_%7B%5Cmathrm%7Bpred%7D%7D) 为预测的协方差矩阵，最后的 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BP%7D) 为修正后的协方差矩阵。这里的 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BH%7D) 的计算可以通过链式法则来生成：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BH%7D+%3D+%5Cfrac%7B%5Cpartial+h%7D%7B%5Cpartial+%5Cbm%7Bx%7D%7D+%5Cfrac%7B%5Cpartial+%5Cbm%7Bx%7D%7D%7B%5Cpartial+%5Cdelta+%5Cbm%7Bx%7D%7D)

其中第一项只需对观测方程进行线性化，第二项，根据我们之前对状态变量的定义，可以得到：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D%5Clabel%7Bkey%7D+%09%5Cfrac%7B%5Cpartial+%5Cbm%7Bx%7D%7D%7B%5Cpartial+%5Cdelta+%5Cbm%7Bx%7D%7D+%3D+%5Cmathrm%7Bdiag%7D%28%5Cbm%7BI%7D_3%2C+%5Cbm%7BI%7D_3%2C+%5Cfrac%7B%5Cpartial+%5Cmathrm%7BLog%7D+%28%5Cbm%7BR%7D%28%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29%29%29%7D%7B%5Cpartial+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%7D%2C+%5Cbm%7BI%7D_3%2C+%5Cbm%7BI%7D_3%2C+%5Cbm%7BI%7D_3%29+%5Cend%7Bequation%7D)

其他几种都是平凡的，只有旋转部分，因为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D) 定义为 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BR%7D) 的右乘，我们用右乘的BCH即可：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D%5Clabel%7Bkey%7D+%09%5Cfrac%7B%5Cpartial+%5Cmathrm%7BLog%7D+%28%5Cbm%7BR%7D%28%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%29%29%29%7D%7B%5Cpartial+%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D%7D+%3D+%5Cbm%7BJ%7D_r%5E%7B-1%7D+%28%5Cbm%7BR%7D%29+%5Cend%7Bequation%7D)

最后，我们可以给每个变量加下标 ![[公式]](https://www.zhihu.com/equation?tex=k) ，表示在 ![[公式]](https://www.zhihu.com/equation?tex=k) 时刻进行状态估计。

### ESKF的误差状态后续处理

在经过预测和更新过程之后，我们修正了误差状态的估计。接下来，只需把误差状态归入名义状态，然后重置ESKF即可。归入部分可以简单地写为：

![[公式]](https://www.zhihu.com/equation?tex=%09%5Cbegin%7Balign%7D+%09%09%5Cbm%7Bp%7D_%7Bk%2B1%7D+%26%3D+%5Cbm%7Bp%7D_k+%2B+%5Cdelta+%5Cbm%7Bp%7D_k+%5C%5C+%09%09%5Cbm%7Bv%7D_%7Bk%2B1%7D+%26%3D+%5Cbm%7Bv%7D_k+%2B+%5Cdelta+%5Cbm%7Bv%7D_k+%5C%5C+%09%09%5Cbm%7BR%7D_%7Bk%2B1%7D+%26%3D+%5Cbm%7BR%7D_k+%5Cmathrm%7BExp%7D%28%5Cdelta+%5Cboldsymbol%7B%5Ctheta%7D_k%29+%5C%5C+%09%09%5Cbm%7Bb%7D_%7Bg%2C+k%2B1%7D+%26%3D+%5Cbm%7Bb%7D_%7Bg%2Ck%7D+%2B+%5Cdelta+%5Cbm%7Bb%7D_%7Bg%2Ck%7D+%5C%5C+%09%09%5Cbm%7Bb%7D_%7Ba%2C+k%2B1%7D+%26%3D+%5Cbm%7Bb%7D_%7Ba%2Ck%7D+%2B+%5Cdelta+%5Cbm%7Bb%7D_%7Ba%2Ck%7D+%5C%5C+%09%09%5Cbm%7Bg%7D_%7Bk%2B1%7D+%26%3D+%5Cbm%7Bg%7D_%7Bk%7D+%2B+%5Cdelta+%5Cbm%7Bg%7D_%7Bk%7D+%09%5Cend%7Balign%7D)

有些文献里也会定义为广义的状态变量加法：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7Bx%7D_%7Bk%2B1%7D+%3D+%5Cbm%7Bx%7D_k+%5Coplus+%5Cdelta+%5Cbm%7Bx%7D_%7Bk%7D)

这种写法可以简化整体的表达式。不过，如果公式里出现太多的广义加减法，可能让人不好马上辨认它们的具体含义，所以本书还是倾向于将各状态分别写开，或者直接用加法而非广义加法符号。

ESKF的重置可以简单地实现为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D%5Clabel%7Bkey%7D+%09%5Cdelta+%5Cbm%7Bx%7D+%3D+%5Cbm%7B0%7D+%5Cend%7Bequation%7D)

同时保留协方差 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbm%7BP%7D) 的估计。

### 小结

本节向大家介绍了SO(3)流形上的ESKF，相比于四元数形式或欧拉角形式，更为简单，无需自定义太多符号。





参考文献

[1]. Xu W, Zhang F. Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter[J]. IEEE Robotics and Automation Letters, 2021, 6(2): 3317-3324.

[2]. Xu W, Cai Y, He D, et al. Fast-lio2: Fast direct lidar-inertial odometry[J]. arXiv preprint arXiv:2107.06829, 2021.

[3]. Madyastha V, Ravindra V, Mallikarjunan S, et al. Extended Kalman filter vs. error state Kalman filter for aircraft attitude estimation[C]//AIAA Guidance, Navigation, and Control Conference. 2011: 6615.

[4]. Sola J. Quaternion kinematics for the error-state Kalman filter[J]. arXiv preprint arXiv:1711.02508, 2017.

[5]. Lupton T, Sukkarieh S. Efficient integration of inertial observations into visual SLAM without initialization[C]//2009 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2009: 1547-1552.