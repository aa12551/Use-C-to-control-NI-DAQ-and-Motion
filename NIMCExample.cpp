////////////////////////////////////////////////////////////////////////////////
// 
//		NIMCExample.c
//		General implemenation file for all NI Motion Control Examples
//
////////////////////////////////////////////////////////////////////////////////

#include "flexmotn.h"
#include <stdio.h>
#include <stdlib.h>

// Error Status
i32 err = -1;

///////////////////////////////////////////////////////////////////////////////
// nimcDisplayError - to print the error description to screen
//
// PARAMETERS: i32 errorCode
//					u16 commandID
//					u16 resourceID
//
// DESCRIPTION: This function takes the error code, command ID and resourceID
//	             and prints the error description to the screen.
//					 If commandID is zero then it just displays the error	- else it
//					 displays the complete description with the function name corresponding
//					 to the commandID
////////////////////////////////////////////////////////////////////////////////


void nimcDisplayError(i32 errorCode, u16 commandID, u16 resourceID) {

	i8 *errorDescription;			//Pointer to i8's -  to get error description
	u32 sizeOfArray;				//Size of error description
	u16 descriptionType;			//The type of description to be printed
	i32 status;						//Error returned by function

	if (commandID == 0) {
		descriptionType = NIMC_ERROR_ONLY;
	}
	else {
		descriptionType = NIMC_COMBINED_DESCRIPTION;
	}


	//First get the size for the error description
	sizeOfArray = 0;
	errorDescription = NULL;//Setting this to NULL returns the size required
	status = flex_get_error_description(descriptionType, errorCode, commandID, resourceID,
		errorDescription, &sizeOfArray);

	//Allocate memory on the heap for the description
	errorDescription = (i8 *)malloc(sizeOfArray + 1);

	sizeOfArray++; //So that the sizeOfArray is size of description + NULL character
	// Get Error Description
	status = flex_get_error_description(descriptionType, errorCode, commandID, resourceID,
		errorDescription, &sizeOfArray);

	if (errorDescription != NULL) {
		printf("\n");
		printf(errorDescription);	//Print description to screen
		free(errorDescription);		//Free allocated memory
	}
	else {
		printf("Memory Allocation Error");
	}
}