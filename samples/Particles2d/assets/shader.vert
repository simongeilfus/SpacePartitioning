#version 410 core

uniform mat4 	ciModelViewProjection;
in vec4			ciPosition;
in float 		ciCustom0;

out float 		vSize;

void main() {
	vSize			= ciCustom0;
	gl_PointSize 	= 3.1 * ciCustom0;
	gl_Position 	= ciModelViewProjection * vec4( ciPosition.xy, 0.0, 1.0 );
} 