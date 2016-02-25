#version 150

uniform mat4	ciModelViewProjection;
uniform mat3	ciNormalMatrix;

in vec4		ciPosition;
in vec3		ciNormal;
in vec3		vInstancePosition;

out highp vec3	Normal;

void main( void )
{
	gl_Position	= ciModelViewProjection * ( ciPosition + vec4( vInstancePosition, 0 ) );
	Normal		= ciNormalMatrix * ciNormal;
}
