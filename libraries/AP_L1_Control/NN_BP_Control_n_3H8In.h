//ann_bp.h//
#ifndef _ANN_BP_H_                 //是否重定义
#define _ANN_BP_H_

#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <windows.h>
#include <ctime>
#include <GCS_MAVLink/GCS.h>
#include <vector>



class Ann_bp
{
public:                                 //explicit关键字只能用于修饰只有一个参数的类构造函数, 它的作用是表明该构造函数是显示的, 而非隐式的
	explicit Ann_bp(int nNIL, int nNOL, int nNHL,int nNHL2,int nNHL3,float umax, float umin);  
	~Ann_bp();
	
	//float* train_forward(float* in);
	float train_forward(float* in);

	void set_bias(float* b1,float* b2,float* b3,float* b4);
	void set_weight(float wt1[5][7], float wt2[7][7],float wt3[7][5],float wt4[5][1]);

private:
	
	int numNodesInputLayer;         //输入层神经元数目    
	int numNodesOutputLayer;
	int numNodesHiddenLayer1;
	int numNodesHiddenLayer2;
	int numNodesHiddenLayer3;

	float umax;                     //输出的上限
	float umin;                     //输出的下限
									
	float*** weights;              //网络权值
	float** bias;                  //网络偏置
	
	float* hidenLayerOutput1;       //隐藏层1各结点的输出值
	float* hidenLayerOutput2;       //隐藏层2各结点的输出值
	float* hidenLayerOutput3;       //隐藏层3各结点的输出值
	float* outputLayerOutput;       //输出层各结点的输出值
	//float* re_n_output;

	//激活函数
	float tansig(float x){return 2 / (1 + exp(-2 * x)) - 1;}
	float logsig(float x) { return 1 / (1 + exp(-1 * x)); }                 
	float purelin(float x) {return (x >=0 ? x : 0);}
};

Ann_bp::Ann_bp(int nNIL, int nNOL, int nNHL1, int nNHL2,int nNHL3,float ua, float ui) :
	numNodesInputLayer(nNIL), numNodesOutputLayer(nNOL),
	numNodesHiddenLayer1(nNHL1), numNodesHiddenLayer2(nNHL2),numNodesHiddenLayer3(nNHL3), umax(ua), umin(ui)
{  
	//创建权值空间,并初始化
	srand((unsigned)time(NULL));      //真随机数
	weights = new float** [4];       //权值三维数组
	
	weights[0] = new float* [numNodesInputLayer];
	for (int i = 0; i < numNodesInputLayer; ++i) {
		weights[0][i] = new float[numNodesHiddenLayer1];
		for (int j = 0; j < numNodesHiddenLayer1; ++j) {
			weights[0][i][j] = (rand() % (2000) / 1000.0 - 1); //-1到1之间，输入层到隐藏层1的权值
		}
	}
	
	weights[1] = new float* [numNodesHiddenLayer1];
	for (int i = 0; i < numNodesHiddenLayer1; ++i) {
		weights[1][i] = new float[numNodesHiddenLayer2];
		for (int j = 0; j < numNodesHiddenLayer2; ++j) {
			weights[1][i][j] = (rand() % (2000) / 1000.0 - 1); //-1到1之间，隐藏层1到隐藏层2的权值
		}
	}

	weights[2] = new float* [numNodesHiddenLayer2];
	for (int i = 0; i < numNodesHiddenLayer2; ++i) {
		weights[2][i] = new float[numNodesHiddenLayer3];
		for (int j = 0; j < numNodesHiddenLayer3; ++j) {
			weights[2][i][j] = (rand() % (2000) / 1000.0 - 1); //-1到1之间，隐藏层2到隐藏层3的权值
		}
	}

	
	weights[3] = new float* [numNodesHiddenLayer3];
	for (int i = 0; i < numNodesHiddenLayer3; ++i) {
		weights[3][i] = new float[numNodesOutputLayer];
		for (int j = 0; j < numNodesOutputLayer; ++j) {
			weights[3][i][j] = (rand() % (2000) / 1000.0 - 1); //-1到1之间，隐藏层3到输出层的权值
		}
	}

	//创建偏置空间，并初始化
	bias = new float* [4];              //偏置二维数组
	
	bias[0] = new float[numNodesHiddenLayer1];
	for (int i = 0; i < numNodesHiddenLayer1; ++i) {
		bias[0][i] = (rand() % (2000) / 1000.0 - 1);           //-1到1之间，隐藏层1到的输入层偏置
	}
	
	bias[1] = new float[numNodesHiddenLayer2];
	for (int i = 0; i < numNodesHiddenLayer2; ++i) {
		bias[1][i] = (rand() % (2000) / 1000.0 - 1);           //-1到1之间，隐藏层2到隐藏层1的偏置
	}

	bias[2] = new float[numNodesHiddenLayer3];
	for (int i = 0; i < numNodesHiddenLayer3; ++i) {
		bias[2][i] = (rand() % (2000) / 1000.0 - 1);           //-1到1之间，隐藏层3到隐藏层2的偏置
	}

	bias[3] = new float[numNodesOutputLayer];
	for (int i = 0; i < numNodesOutputLayer; ++i) {
		bias[3][i] = (rand() % (2000) / 1000.0 - 1);           //-1到1之间，输出层到隐藏层3的偏置
	}


	//创建隐藏层各结点的输出值空间
	hidenLayerOutput1 = new float[numNodesHiddenLayer1];        //一维数组
	hidenLayerOutput2 = new float[numNodesHiddenLayer2]; 
	hidenLayerOutput3 = new float[numNodesHiddenLayer3]; 

    //创建输出层各结点的输出值空间
	outputLayerOutput = new float[numNodesOutputLayer];
	//float* re_n_output = new float[numNodesOutputLayer];
     
}

void Ann_bp::set_bias(float* b1,float* b2,float* b3,float* b4)
{
	for (int i = 0; i < numNodesHiddenLayer1; i++)
		bias[0][i] = b1[i];
				
	for (int i = 0; i < numNodesHiddenLayer2; i++)
		bias[1][i] = b2[i];

	for (int i = 0; i < numNodesHiddenLayer3; i++)
		bias[2][i] = b3[i];
	
    for (int i = 0; i < numNodesOutputLayer; i++)
		bias[3][i] = b4[i];	

}

	
void Ann_bp::set_weight(float wt1[5][7], float wt2[7][7],float wt3[7][5],float wt4[5][1])
{

    for (int i = 0; i < numNodesInputLayer; i++)
		for (int j = 0; j < numNodesHiddenLayer1; j++)
			weights[0][i][j] = wt1[i][j];

	for (int i = 0; i < numNodesHiddenLayer1; i++)
		for (int j = 0; j < numNodesHiddenLayer2; j++)
			weights[1][i][j] = wt2[i][j];

    for (int i = 0; i < numNodesHiddenLayer2; i++)
		for (int j = 0; j < numNodesHiddenLayer3; j++)
			weights[2][i][j] = wt3[i][j];

	for (int i = 0; i < numNodesHiddenLayer3; i++)
		for (int j = 0; j < numNodesOutputLayer; j++)
			weights[3][i][j] = wt4[i][j];	
}

float Ann_bp::train_forward(float* inputMat) {

   //计算隐藏层1各结点的输出
	for (int i = 0; i < numNodesHiddenLayer1; ++i) {
		float z = 0.0;
		for (int j = 0; j < numNodesInputLayer; ++j) {
			z += inputMat[j] * weights[0][j][i];
		}
		z += bias[0][i];
		hidenLayerOutput1[i] = tansig(z);
	}

	gcs().send_text(MAV_SEVERITY_INFO, "H1= %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", (float)hidenLayerOutput1[0],(float)hidenLayerOutput1[1],(float)hidenLayerOutput1[2],(float)hidenLayerOutput1[3],(float)hidenLayerOutput1[4],(float)hidenLayerOutput1[5],(float)hidenLayerOutput1[6]);


   //计算隐藏层2各结点的输出
	for (int i = 0; i < numNodesHiddenLayer2; ++i) {
		float z = 0.0;
		for (int j = 0; j < numNodesHiddenLayer1; ++j) {
		    z += hidenLayerOutput1[j] * weights[1][j][i];
		}
		z += bias[1][i];
		hidenLayerOutput2[i] = logsig(z);
	 }

	gcs().send_text(MAV_SEVERITY_INFO, "H2= %.2f,%.2f,%.2f,%.2f,%.2f", (float)hidenLayerOutput2[0],(float)hidenLayerOutput2[1],(float)hidenLayerOutput2[2],(float)hidenLayerOutput2[3],(float)hidenLayerOutput2[4]);
	


	 //计算隐藏层3各结点的输出
	for (int i = 0; i < numNodesHiddenLayer3; ++i) {
		float z = 0.0;
		for (int j = 0; j < numNodesHiddenLayer2; ++j) {
			z += hidenLayerOutput2[j] * weights[2][j][i];
		}
		z += bias[2][i];
		hidenLayerOutput3[i] = tansig(z);
	  }

	gcs().send_text(MAV_SEVERITY_INFO, "H3= %.2f,%.2f,%.2f,%.2f,%.2f", (float)hidenLayerOutput3[0],(float)hidenLayerOutput3[1],(float)hidenLayerOutput3[2],(float)hidenLayerOutput3[3],(float)hidenLayerOutput3[4]);
		


	//计算输出层结点的输出值
	for (int i = 0; i < numNodesOutputLayer; ++i) {
		float z = 0.0;
		for (int j = 0; j < numNodesHiddenLayer3; ++j) {
			z += hidenLayerOutput3[j] * weights[3][j][i];
		}
		z += bias[3][i];
		outputLayerOutput[i] = tansig(z);
	}

	gcs().send_text(MAV_SEVERITY_INFO, "O= %.4f", (float)outputLayerOutput[0]);


	//反归一化
		float re_n_output = (outputLayerOutput[0] - (-1)) * (umax-umin)/(1.0-(-1)) + umin;
	
		return re_n_output;
}

Ann_bp::~Ann_bp()
{                                                       //数组空间层层释放
	//释放权值空间
	for (int i = 0; i < numNodesInputLayer; ++i)
		delete[] weights[0][i];
	for (int i = 0; i < numNodesHiddenLayer1; ++i)
		delete[] weights[1][i];
	for (int i = 0; i < numNodesHiddenLayer2; ++i)
		delete[] weights[2][i];
	for (int i = 0; i < numNodesHiddenLayer3; ++i)
		delete[] weights[3][i];
	for (int i = 0; i < 4; ++i)
		delete[] weights[i];
	delete[] weights;

	//释放偏置空间
	for (int i = 0; i < 4; ++i)
		delete[] bias[i];
	delete[] bias;

	//释放输出空间
	delete[] hidenLayerOutput1;
	delete[] hidenLayerOutput2;
	delete[] hidenLayerOutput3;
	delete[] outputLayerOutput;

}
#endif
