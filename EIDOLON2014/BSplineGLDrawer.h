//------------------------------------------------------------
/// \file	Main.cpp
/// \author	Rob Bateman
/// \date	9-feb-2005
/// \brief	This example will demonstrate how to clamp a curve
/// 		to it's end points. It is also the fundamental brigde
/// 		to be able to understand how the cox-de-boor
/// 		algorithm works.
///
//------------------------------------------------------------
#pragma once
#include <stdlib.h>
#include <gl\GL.h>
#include <gl\GLU.h>
#include <GLFW\glfw3.h>
#include <vector>
#include "ARS_MathHelper.h"
#include "ARS_BSpline.h"

// the level of detail for the curve
unsigned int LOD=20;

//------------------------------------------------------------	GetPoint()
// To handle the clamped curves i imagine the curve to have 3 
// extra control points at the beginning of the curve, and 3 extra
// at the end. To simply the iteration of the points, i start
// the indices at -3. Obviously if i use that value as an array
// index it will crash horribly. Instead i use a function that
// returns either the first, or last point if the index requested
// is out of range. This basically simplifies our curve calculation
// greatly!!
//
Vector3f GetPoint(vector<Vector3f>* controlPoints, int i) {
	// return 1st point
	if (i<0) {
		return	controlPoints->at(0);
	}
	// return last point
	if (i<controlPoints->size())
		return controlPoints->at(i);

	return controlPoints->at(controlPoints->size()-1);
}

//------------------------------------------------------------	OnKeyPress()
void DrawSpline(ARS_BSpline* spline, Vector3f desloc, Vector3f curveColor, Vector3f cuColor, Vector3f vertexColor) {
	
	glColor3f(cuColor.x,cuColor.y,cuColor.z);
	glPointSize(3);
	glPushMatrix();

	glTranslatef(desloc.x,desloc.y,desloc.z);
	
	// draw curve hull
	glBegin(GL_LINE_STRIP);
	for(int i=0;i!=spline->controlPoints.size();++i) {
		Vector3f pt = spline->controlPoints.at(i);
		glVertex3f( pt.x, pt.y, pt.z );
	}
	glEnd();


	glColor3f(curveColor.x,curveColor.y,curveColor.z);

	// begin drawing our curve
	glBegin(GL_LINE_STRIP);

	float step = 1.0f / LOD;
	float i = 0.0f;
	while(i <= 1.0f){
			Vector3f pos = spline->getPosition(i, 3);

			// specify the point
			glVertex3f( pos.x,pos.y,0 );

			i += step;
	}
	glEnd();

	glColor3f(vertexColor.x,vertexColor.y,vertexColor.z);
	// draw CV's
	glBegin(GL_POINTS);
	for(int i=0;i!=spline->controlPoints.size();++i) {
		Vector3f pt = spline->controlPoints.at(i);
		glVertex3f( pt.x, pt.y, pt.z );
	}
	glEnd();

	glPopMatrix();
}