#version 150

uniform mat4	ciModelViewProjection;

in vec4		ciPosition;
in vec3		vInstancePosition;


void main( void )
{
	gl_Position	= ciModelViewProjection * ( ciPosition + vec4( vInstancePosition, 0 ) );
}
