#version 150

out vec4 oColor;

void main( void )
{
	vec2 pos = gl_PointCoord.st - vec2( 0.5f );
	oColor = vec4( vec3( smoothstep( 0.6f, 0.1f, length( pos ) ) ), 1.0 );
}