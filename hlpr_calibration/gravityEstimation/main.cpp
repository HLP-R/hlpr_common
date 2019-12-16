#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>

using namespace std;
int main()
{
        cout << endl << endl;
        cout << "========================================================" << endl;
        cout << "=====  Estimate Gravity Parameters for Jaco2 arm   =====" << endl;
        cout << "========================================================" << endl;
        cout << "code: Reza Ahmadzadeh (IRIM, 2017)." << endl;
        cout << "WARNING: Read the documentation before running this code!" << endl << endl;

        int result;
        int programResult = 0;
        int devicesCount;
        //Handle for the library's command layer.
        void * commandLayer_handle;
        //Function pointers to the functions we need
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyGetAngularCommand)(AngularPosition &);
        int(*MyRunGravityZEstimationSequence7DOF)(ROBOT_TYPE type, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF]); //for 7DOF
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);

        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MyRunGravityZEstimationSequence7DOF = (int(*)(ROBOT_TYPE, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence7DOF");


        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");

        if ( (MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL))
        {
                cout << "* * *  Error during initialization!  * * *" << endl;
                programResult = 0;
        }
        else
        {
                cout << "Initialization completed." << endl << endl;
                result = (*MyInitAPI)();

                KinovaDevice list[MAX_KINOVA_DEVICE];

                 devicesCount = MyGetDevices(list, result);
                 if (devicesCount == 0)
                 {
                     cout << "\n WARNING : The robot is off or is not in the loop!" << endl;
                     return 0;
                 }
                 else if (devicesCount > 1)
                 {
                     cout << "\n WARNING: There are multiple robots connected. This process is considered for a single robot." << endl;
                    return 0;
                 }

                 cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ") (" << list[0].DeviceType << ")" << endl;

                 MySetActiveDevice(list[0]);  //Setting the current device as the active device.

                int resultComm;
                MySwitchTrajectoryTorque(POSITION);
                cout << "set the robot in position trajectory mode." << endl;
                AngularPosition DataCommand;	                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                cout << "Communication result :" << resultComm << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        ROBOT_TYPE type = SPHERICAL_7DOF_SERVICE; 
                        float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF];
                        // Run identification sequence
                        // CAUTION READ THE FUNCTION DOCUMENTATION BEFORE
                        cout << "This process takes about 15minutes to complete." << endl;
                        cout << "During the estimation you should watch the robot carefully and be ready to take action." << endl;
                        cout << "WARNING: Please cancle this program if you don't know what to expect" << endl;
                        MyRunGravityZEstimationSequence7DOF(type, OptimalzParam);
                }
                cout << endl << "Estimation process has finished! Set the parameters using the command {setparam}." << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}

