
#include "move.h"
#include "math.h"
#include "iostream"
void move(int pos, u8 axis, i32 velocity)
{
	u8	boardID;					// Board identification number
	u16 csr = 0;				// Communication status register
	u16 axisStatus;			// Axis status
	u16 moveComplete;
	i32 position;
	//Variables for modal error handling
	u16 commandID;				// The commandID of the function
	u16 resourceID;			// The resource ID
	i32 errorCode;				// Error code

	///////////////////////////////
	// Set the board ID
	boardID = 1;

	// Initialize the controller
	err = flex_initialize_controller(boardID, NULL);
	CheckError;

	// Set the velocity for the move (in counts/sec)
	err = flex_load_velocity(boardID, axis, velocity, 0xFF);
	CheckError;

	// Set the acceleration for the move (in counts/sec^2)
	err = flex_load_acceleration(boardID, axis, NIMC_ACCELERATION, 10000, 0xFF);
	CheckError;

	// Set the deceleration for the move (in counts/sec^2)
	err = flex_load_acceleration(boardID, axis, NIMC_DECELERATION, 10000, 0xFF);
	CheckError;

	// Set the jerk - scurve time (in sample periods)
	err = flex_load_scurve_time(boardID, axis, 1000, 0xFF);
	CheckError;

	// Set the operation mode
	err = flex_set_op_mode(boardID, axis, NIMC_RELATIVE_POSITION);
	CheckError;

	// Load Position
	err = flex_load_target_pos(boardID, axis, pos, 0xFF);
	CheckError;

	// Start the move
	err = flex_start(boardID, axis, 0);
	CheckError;

	do {
		//Read the current position of axis
		err = flex_read_pos_rtn(boardID, axis, &position);
		CheckError;

		err = flex_read_axis_status_rtn(boardID, axis, &axisStatus);
		CheckError;

		//Read the Communication Status Register - check the 
		//modal error bit
		err = flex_read_csr_rtn(boardID, &csr);
		CheckError;

		if (csr & NIMC_MODAL_ERROR_MSG)
		{
			flex_stop_motion(boardID, NIMC_VECTOR_SPACE1, NIMC_DECEL_STOP, 0);//Stop the Motion
			err = csr & NIMC_MODAL_ERROR_MSG;
			CheckError;
		}

	} while (!(axisStatus & (NIMC_MOVE_COMPLETE_BIT | NIMC_AXIS_OFF_BIT))); //Test against the move complete bit
	std::cout << "Axis " << axis << " position: " << position << std::endl;						  //	or the axis off bit
	std::cout << "Finished" << std::endl;;

	return;
		// Exit the Application


	/////////////////////////////////////////////////////////////////////////
	// Error Handling
	//
	nimcHandleError; //NIMCCATCHTHIS:

	// Check to see if there were any Modal Errors
	if (csr & NIMC_MODAL_ERROR_MSG) {
		do {
			//Get the command ID, resource and the error code of the modal
			//	error from the error stack on the board
			flex_read_error_msg_rtn(boardID, &commandID, &resourceID, &errorCode);
			nimcDisplayError(errorCode, commandID, resourceID);

			//Read the Communication Status Register
			flex_read_csr_rtn(boardID, &csr);

		} while (csr & NIMC_MODAL_ERROR_MSG);
	}
	else		// Display regular error 
		nimcDisplayError(err, 0, 0);
	return;		// Exit the Application
}

void stop()
{
	u8 boardID = 1;
	err = flex_stop_motion(boardID, 1, NIMC_DECEL_STOP, 0);
	err = flex_stop_motion(boardID, 2, NIMC_DECEL_STOP, 0);
	err = flex_stop_motion(boardID, 4, NIMC_DECEL_STOP, 0);
}

