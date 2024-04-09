//
// Created by 91418 on 2024/3/9.
//
#include "grap_action.h"
int main(int argc,char**argv)
{
	EndEffector_Motion em;
	em.Enable();
	if (argc>=2){
		int signal = atoi(argv[1]);
		switch (signal){
		case 0:
			cout<<"Close!"<<endl;
			em.grap_tool(GRAP_CLOSE);
			break;
		case 1:
			cout<<"Open!"<<endl;
			em.grap_tool(GRAP_OPEN);
			break;
		}
	}

	//    EndEffector_Position ep1;
	//		EndEffector_Torque et;
	//		et.Enable();
	//		et.Motion({0,0,-500,-500});

	//      Sleep(2000);
	//   	em.ftmr(ROTATE_FORWARD);
	//      Sleep(2000);
//	em.ftmr(ROTATE_BACKWARD);
	//	em.grap_tool(GRAP_CLOSE);
	//	em.grap_tool(GRAP_CLOSE);
	system("pause");
	Sleep(20);
	return 0;
}