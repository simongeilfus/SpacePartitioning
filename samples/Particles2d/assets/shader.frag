#version 410 core

in float	vSize;
out vec4	oColor;

void main() {
	// use gl_PointCoord to make a circle
	vec2 uv 		= gl_PointCoord;
	float center 	= length( uv - vec2( 0.5 ) );
	// if we don't tweak the smoothstep using the size of the point
	// we will have different sharpness/smoothness resulting in larger
	// points being blurry or smaller being too aliased.
	float circle 	= smoothstep( 0.5, 0.05 + vSize * 0.06, center );
    oColor 			= vec4( circle );
}