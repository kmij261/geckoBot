//#include "tracjectoryPlanning.h"

//const u16 DELAY_CONST = 20;											//步间最大延时
//const double BASE_X = 70.0, BASE_Y = 40.0, BASE_Z = -35.0;			//起始位置(mm)
//const double link1 = 70.0, link2 = 40.0, link3 = 35.0;				//连杆长度(mm)
//const double pi = 3.14159265359;									//π
//Coor3d C[4];														//四个脚掌末端位置
//Joints J[4];														//四条腿关节角度
//double AngleBuf[12] = {0.0};										//角度数组
//State state;														//运动状态变量

///*
////#define IN_DEBUG

//////////////////////////////////////////////////////////////////////////////////
///// @功能 系统初始化
///// @参数 init_mode:初始化模式；s：运动学参数结构体
///// @返回 0：初始化成功，LED常亮；非0：失败，LED闪烁
//////////////////////////////////////////////////////////////////////////////////
//u8 gecko_init(LOCO_MODE init_mode, State s)
//{
//	u8 ret = 0;
//	delay_init(168);  							//初始化延时函数
//	usart_init(115200);							//初始化串口波特率为115200
//    servo_init(init_mode);						//初始化舵机，并运动到初始状态
//	LED_Init();									//初始化应用指示灯
//		
//	set_gait(s.gait);
//	reset_t();
//	
//	ret += set_speed(s.speed);
//	ret += set_peroid(s.T);
//	ret += set_step(s.step);
//	ret += set_step_len(s.step_len);
//	ret += set_step_height(s.step_height);

//	//如果初始化错误
//	if(ret)
//	{
//		err_handle(ERR_INIT_FAILED);
//		CLR_ERR;
//		return ret;
//	}
//	
//	//如果初始化成功，LED常亮
//	else
//		LED = 0;
//	
//	
//	return 0;

//}


//////////////////////////////////////////////////////////////////////////////////
///// @功能 运动到下一个位置点
///// @参数 无
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void single_step(void)
//{
//	u8 speed = abs(state.speed);
//	
//	u8 i;
//	
//	//静止
//	if(state.speed == 0)
//		return;
//	
//	//计算关节角度值并写入数组
//	joints_update();
//	update_angle_buf();
//	
//	//调用舵机控制
//	servo_ctrl_degree(AngleBuf);
//	
//#ifdef IN_DEBUG
//		
//	printf("%d \r\n", state.t);
//	
//	for(i=0; i<12; i++)
//	{
//		printf("%3.2f  ", AngleBuf[i]);
//	}
//	
//	printf("\r\n");

//#endif	
//	
//	//更新t
//	t_update();
//	
//	//用于调节步频
////	delay_ms(DELAY_CONST / speed);
//	
//}


//////////////////////////////////////////////////////////////////////////////////
///// @功能 根据步态生成足端位置，并计算运动学逆解
///// @参数 无
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void joints_update(void)
//{
//	u8 i;
//	
//	//根据步态生成当前足端位置
//	gait_gen();
//	
//	//运动学逆解得到关节角度值
//	for(i=0; i<4; i++)
//		J[i] = ikine(C[i]);
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 生成步态散点
///// @参数 无
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void gait_gen(void)
//{
//	//对角步态
//	if(state.gait == GAIT_BI)
//		bipod_gait(state.t);
//	
//	//三角步态
//	if(state.gait == GAIT_TRI)
//		tripod_gait(state.t);
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 生成对角步态的一系列散点
///// @参数 t：当前序列值（t从0到T为一个周期）
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void bipod_gait(int t)
//{
//	double T = state.T;								//周期点数
//	double L = state.step_len;						//步长
//	double H  = state.step_height;					//抬腿高度
//	t = (double)t;									//转换成double
//	
//	if(t < 0.25*T)
//	{
//		//左前腿
//		C[LF].x = BASE_X;
//		C[LF].y = -2.0*L*t / T + BASE_Y;
//		C[LF].z = BASE_Z;
//		
//		//右前腿
//		C[RF].x = BASE_X;
//		C[RF].y = 2.0*L*t / T + BASE_Y;
//		C[RF].z = H*sin(0.5*pi*(4*t / T + 1)) + BASE_Z;
//		
//		//左后腿
//		C[LH].x = BASE_X;
//		C[LH].y = -2.0*L*t / T + BASE_Y;
//		C[LH].z = H*sin(0.5*pi*(4*t / T + 1)) + BASE_Z;
//		
//		//右后腿
//		C[RH].x = BASE_X;
//		C[RH].y = 2.0*L*t / T + BASE_Y;
//		C[RH].z = BASE_Z;
//		
//		
//	}
//	
//	else if(t < 0.75*T)
//	{
//		//左前腿
//		C[LF].x = BASE_X;
//		C[LF].y = 2.0*L*(t-T/4.0) / T - L/2.0 + BASE_Y;
//		C[LF].z = H*sin(2.0*pi*(t-T/4.0) / T) + BASE_Z;
//		
//		//右前腿
//		C[RF].x = BASE_X;
//		C[RF].y = -2.0*L*(t - T/4)/T + L/2.0 + BASE_Y;
//		C[RF].z = BASE_Z;;
//		
//		//左后腿
//		C[LH].x = BASE_X;
//		C[LH].y = 2.0*L*(t - T/4)/T - L/2.0 + BASE_Y;
//		C[LH].z = BASE_Z;;
//		
//		//右后腿
//		C[RH].x = BASE_X;
//		C[RH].y = -2.0*L*(t-T/4.0) / T + L/2.0 + BASE_Y;
//		C[RH].z = H*sin(2.0*pi*(t-T/4.0) / T) + BASE_Z;
//	}
//	
//	else if(t <= T)
//	{
//		//左前腿
//		C[LF].x = BASE_X;
//		C[LF].y = L/2.0*(1 - (4*t - 3*T)/T) + BASE_Y;
//		C[LF].z = BASE_Z;
//		
//		//右前腿
//		C[RF].x = BASE_X;
//		C[RF].y = -2.0*L*(T - t)/T + BASE_Y;
//		C[RF].z = H*sin(2.0*pi * ((t - 3.0*T/4.0) / T)) + BASE_Z;
//		
//		//左后腿
//		C[LH].x = BASE_X;
//		C[LH].y = 2.0*L*(T - t)/T + BASE_Y;
//		C[LH].z = H*sin(2.0*pi * ((t - 3.0*T/4.0) / T)) + BASE_Z;
//		
//		//右后腿
//		C[RH].x = BASE_X;
//		C[RH].y = -L/2.0*(1 - (4*t - 3*T)/T) + BASE_Y;
//		C[RH].z = BASE_Z;
//		
//	}
//	

//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 生成三角步态的一系列散点
///// @参数 t：当前序列值（t从0到T为一个周期）
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void tripod_gait(int t)
//{
//	
//}


//////////////////////////////////////////////////////////////////////////////////
///// @功能 将四条腿的各关节角度值，送入关节角度数组
///// @参数 无
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void update_angle_buf(void)
//{
//	u8 i = 0;
//	
//	for(i=0; i<4; i++)
//	{
//		
//		set_angle_buf(J[i].j1, (3*i + 0));
//		set_angle_buf(J[i].j2, (3*i + 1));
//		set_angle_buf(J[i].j3, (3*i + 2));
//	}
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 按照元素索引改变其值
///// @参数 val：新的浮点值； index：元素位置
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void set_angle_buf(double val, u8 index)
//{
//	AngleBuf[index] = val;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 设定运动参数的中的速度，速度决定了两个位置点之间的时间间隔
///// @参数 speed;速度值，正表示正向。负值则反向，0表示静止
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//u8 set_speed(int speed)
//{
//	//速度范围 （-10 - 10）
//	if(speed > 10)
//		return 1;
//	
//	//速度范围 （-10 - 10）
//	if(speed < -10)
//		return 1;
//	
//	state.speed = speed;
//	
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 将运动学参数中的t值清零
///// @参数 无
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void reset_t(void)
//{
//	state.t = 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 设定运动参数的中的步态
///// @参数 gait：步态对应的需要，参见GAIT枚举
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void set_gait(GAIT gait)
//{
//	state.gait = gait;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 根据运动学参数中的speed和step，更新t
///// @参数 无
///// @返回 无
//////////////////////////////////////////////////////////////////////////////////
//void t_update(void)
//{
//	//正向运动
//	if(state.speed > 0)
//	{
//		state.t += state.step;
//		if(state.t >= state.T)
//			state.t -= state.T;
//	}
//	
//	//反向运动
//	if(state.speed < 0)
//	{
//		state.t -= state.step;
//		if(state.t <= 0)
//			state.t += state.T;
//	}
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 设定运动学参数中的step
///// @参数 step：t每次增加或减少的值
///// @返回 0：设定完成； 非0：参数超出范围
//////////////////////////////////////////////////////////////////////////////////
//u8 set_step(u8 step)
//{
//	if(step < 1)
//		return 1;
//	if(step > state.T/40)
//		return 1;
//	
//	state.step = step;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 设定运动学参数中的周期
///// @参数 peroid：每个步态周期的位置点数
///// @返回 0：设定完成； 非0：参数超出范围
//////////////////////////////////////////////////////////////////////////////////
//u8 set_peroid(double peroid)
//{
//	if(peroid < 100)
//		return 1;
//	
//	if(peroid > 500)
//		return 1;
//	
//	state.T = peroid;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 设定运动学参数中的步长step_len
///// @参数 len：一个周期运动的步长，单位mm
///// @返回 0：设定完成； 非0：参数超出范围
//////////////////////////////////////////////////////////////////////////////////
//u8 set_step_len(double len)
//{
//	if(len <=0)
//		return 1;
//	
//	if(len > 80)
//		return 1;
//	
//	state.step_len = len;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 设定运动学参数中的抬腿高度（相对），step_height
///// @参数 h：每次抬腿的最大高度，单位mm
///// @返回 0：设定完成； 非0：参数超出范围
//////////////////////////////////////////////////////////////////////////////////
//u8 set_step_height(double h)
//{
//	if(h <= 0)
//		return 1;
//	
//	if(h > 35.0)
//		return 1;
//	
//	state.step_height = h;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 根据足端三维坐标计算运动学逆解
///// @参数 c:足端三维坐标
///// @返回 单腿的三个关节角度值
//////////////////////////////////////////////////////////////////////////////////
//Joints ikine(Coor3d c)
//{
//	double x = c.x, y = c.y, z = c.z;
//	double exp1, exp2, exp3;
//	Joints j;
//	
//	exp1 = sqr(x) + sqr(y) + sqr(z) + sqr(link1) - sqr(link2) - sqr(link3);
//	exp2 = 2*link1*(sqrt(sqr(x) + sqr(y) + sqr(z) - sqr(link3)));
//	exp3 = y / sqrt(sqr(x) + sqr(y) + sqr(z) - sqr(link3));
//	
//	//计算关节角度值（弧度）
//	j.j1 = asin(-link3 / sqrt(sqr(x) + sqr(z))) - atan(z/x);
//	j.j2 = asin(exp1 / exp2) - acos(exp3);
//	j.j3 = asin((sqr(link1)+sqr(link2)+sqr(link3)-sqr(x)-sqr(y)-sqr(z)) / (2*link1*link2));
//	
//	//弧度转角度
//	j.j1 *= -180.0/pi;
//	j.j2 *= 180.0/pi;
//	j.j3 *= 180.0/pi;
////	
//	return j;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @功能 计算浮点数的平方
///// @参数 x:底数
///// @返回 结果
//////////////////////////////////////////////////////////////////////////////////
//double sqr(double x)
//{
//	return x*x;
//}

//*/
