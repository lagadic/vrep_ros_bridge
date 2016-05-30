#ifndef __CACCESS_H__
#define __CACCESS_H__

#include "porting.h"
#include "v_repLib.h"

static const char pluginName[] = "RosBridge";


class CustomDataHeaders {
public:

    /**
     * Quadrotor control mode defines.
     */
    typedef enum {
        DIRECT = 0, ///< Set propeller forces and torques.
        INTERNAL = 1 ///< Set total thrust and desired roll, pitch and yaw rate. An internal controller is used for attitude regulation.
    } QuadrotorCtrlMode;

    /**
     * Manipulator control mode defines.
     */
    typedef enum {
        TF_POSITION = 0, ///< Set joint position.
        TF_VELOCITY = 1, ///< Set joint velocity.
        TF_EFFORT = 2,
        MOT_VELOCITY = 3,
        PASSIVE_MODE = 4,
        PASSIVE_MODE_VELOCITY = 5,
        IGNORE_MODE = 20

    } ManipulatorCtrlMode;

    /**
     * The identifier of this plugin custom data.
     * Do [num2str(uint8('L')) num2str(uint8('a')) num2str(uint8('g')) num2str(uint8('a'))] in Matlab to generate this number.
     */
    const static unsigned int DEVELOPER_DATA_HEADER = 769710397;

    ///@name Quadrotor defines
    ///@{
	///The main identifier of a Quadrotor object.
    const static unsigned int QUADROTOR_DATA_MAIN=10;
	///The main identifier of a Telekyb Quadrotor object.
    const static unsigned int QUADROTOR_TK_DATA_MAIN=20;
	///The identifier of motor[0] joint.
    const static unsigned int QUADROTOR_DATA_MOTOR_0=0;
    ///The identifier of motor[1] joint.
    const static unsigned int QUADROTOR_DATA_MOTOR_1=1;
    ///The identifier of motor[2] joint.
    const static unsigned int QUADROTOR_DATA_MOTOR_2=2;
    ///The identifier of motor[3] joint.
    const static unsigned int QUADROTOR_DATA_MOTOR_3=3;
    /// The identifier of the robot CoM.
    const static unsigned int QUADROTOR_DATA_COM=11;
	/// The propeller torque to force ratio.
    const static unsigned int QUADROTOR_DATA_TF_RATIO=12;
    /// The propeller torque to force ratio.
    const static unsigned int QUADROTOR_DATA_CTRL_MODE = 13;
    ///@}

    ///@name Imu defines
    ///@{
	/// The main identifier of a IMU object.
    const static unsigned int IMU_DATA_MAIN=200;
	/// The identifier of the IMU mass.
    const static unsigned int IMU_DATA_MASS=201;
	/// The identifier of the IMU force sensor.
    const static unsigned int IMU_DATA_FORCE=202;
 	/// The frequency at which the IMU readings must be published.
    const static unsigned int IMU_DATA_FREQ= 203;
	/// The cut-off frequency of the acceleration low-pass filters.
    const static unsigned int IMU_DATA_CUTOFF=204;
    ///@}

    ///@name Force sensor defines
    ///@{
	/// The main identifier of a FORCE_SENSOR object.
    const static unsigned int FORCE_SENSOR_DATA_MAIN= 250;
    ///@}

    ///@name Camera defines
    ///@{
	/// The main identifier of a Camera object.
    const static unsigned int CAMERA_DATA_MAIN = 300;
    /// The frequency at which the camera images must be published.
    const static unsigned int CAMERA_DATA_FREQ = 301;
	/// Set to 0 if the camera is gray-scale; set to any other value otherwise.
    const static unsigned int CAMERA_DATA_RGB = 302;
	/// Set to 0 if the depth should be ignored; set to any other value otherwise.
    const static unsigned int CAMERA_DATA_HAS_DEPTH = 303;
	/// Set to 0 if the image should be published with a 8-bit encoding; set to any other value to use a 32 bit float encoding instead.
    const static unsigned int CAMERA_DATA_USE_FLOAT = 304;
    ///@}

    ///@name Object Pose defines
    ///@{
	/// The main identifier of a Pose measurement object.
    const static unsigned int OBJ_POSE_DATA_MAIN=400;
    ///@}

    ///@name Object Twist defines
    ///@{
    /// The main identifier of a Twist measurement object.
    const static unsigned int OBJ_TWIST_DATA_MAIN=500;
    ///@}

    ///@name Manipulator defines
    ///@{
    /// The main identifier of a manipulator object.
    const static unsigned int MANIPULATOR_DATA_MAIN = 600;
    /// The frequency for joint status publishing.
    const static unsigned int MANIPULATOR_DATA_FREQ = 601;
    /// The identifier of a joint. The joint ID must be specified as a unique integer.
    const static unsigned int MANIPULATOR_DATA_JOINT = 602;
    /// The manipulator control mode.
    const static unsigned int MANIPULATOR_DATA_CTRL_MODE = 603;
    ///@}

    ///@name Contact sensor defines
    ///@{
    /// The main identifier of a CONTACT object.
    const static unsigned int CONTACT_DATA_MAIN= 650;
    ///@}



	/**
	 * Make all defined headers available in Lua.
	 * Please update the implementation of this function when you add a new header in \p CustomDataHeaders.
	 */
    static void registerCustomDataHeaders();
};


#include <vector>

/**
 * This class contains helper functions to insert/extract items to/from scene objects.
 * It assumes that the developer stores data under the DEVELOPER_HEADER in the form:
 * item1ID,item1DataLengthInBytes,item1Data,item2ID,item2DataLengthInBytes,item2Data, etc.
 */
class CAccess  
{
public:
	CAccess();
	virtual ~CAccess();

	/**
	 * Inserts data under the \p dataName item.
	 * @param buffer The buffer in which the data must be inserted.
	 * @param dataName The ID of the data to be inserted (must be in \a CustomDataHeaders).
	 * @param data The data value.
	 */
	static void insertSerializationData(std::vector<unsigned char>& buffer,int dataName,const std::vector<unsigned char>& data);

	/** Extracts the data saved under the \p dataName item. Note that the data is erased from the input data \p buffer.
	 * @param buffer The buffer from which the data must be extracted.
	 * @param dataName The ID of the data to be extracted (must be in \a CustomDataHeaders).
	 * @param data The extracted data value.
	 * @return true if the operation was successful, false otherwise.
	 */
	static bool extractSerializationData(std::vector<unsigned char>& buffer,int dataName,std::vector<unsigned char>& data);

	/**
	 * Returns the location and size of the specified field in the custom data.
	 * @param buffer Vector containing the custom data.
	 * @param dataName The ID of the required data (must be in \a CustomDataHeaders).
	 * @param theSize The size of the specified field.
	 * @return The position (i.e. the index) of the specified data in the \p buffer.
	 */
	static int getDataLocationAndSize(const std::vector<unsigned char>& buffer,int dataName,int& theSize);

	/**
	 * Insert an integer in the custom data.
	 * @param buffer The custom data.
	 * @param data The integer to be inserted.
	 */
	static void push_int(std::vector<unsigned char>& buffer, const int data);

	/**
     * Insert a float in the custom data.
     * @param buffer The custom data.
     * @param data The float to be inserted.
     */
	static void push_float(std::vector<unsigned char>& buffer, const float data);

	/**
     * Insert a float array in the custom data.
     * @param buffer The custom data.
     * @param data The float array to be inserted.
     */
	static void push_float(std::vector<unsigned char>& buffer, const std::vector<float> data);

    /**
     * Extract an integer from the custom data. Returns 0 if the \p buffer is smaller than sizeof(int).
     * If the buffer is larger than sizeof(int) only the last sizeof(int) bytes are considered.
     * @param buffer The custom data.
     * @return The extracted integer.
     */
	static int pop_int(std::vector<unsigned char>& buffer);

    /**
     * Extract a float from the custom data. Returns 0 if the \p buffer is smaller than sizeof(float).
     * If the buffer is larger than sizeof(float) only the last sizeof(float) bytes are considered.
     * @param buffer The custom data.
     * @return The extracted float.
     */
	static float pop_float(std::vector<unsigned char>& buffer);

    /**
     * Extract a float array from the custom data. Returns 0 if the \p buffer is smaller than sizeof(float)*N.
     * If the buffer is larger than sizeof(float)*N only the last sizeof(float)*N bytes are considered.
     * @param buffer The custom data.
     * @param n Number of floats to pop.
     * @return The extracted float.
     */
	static std::vector<float> pop_float(std::vector<unsigned char>& buffer, const unsigned int n);

};

#endif //ndef __CACCESS_H__
