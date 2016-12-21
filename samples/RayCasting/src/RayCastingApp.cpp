#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CameraUi.h"

#include "sp/BVH.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// Basic Triangle Class
// The BVH class needs to have access to 3 functions; getBounds, getCentroid and intersect
class Triangle {
public:
	ci::AxisAlignedBox getBounds() const { return AxisAlignedBox( min( a, min( b, c ) ), max( a, max( b, c ) ) ); }
	vec3 getCentroid() const { return ( a + b + c ) / 3.0f; }
	bool intersect( const Ray& r, float* dist ) const { return r.calcTriangleIntersection( a, b, c, dist ); }
	vec3 a, b, c;
};

// If the class doesn't provide those functions you can specialize BVHObjectTraits to specify your own
//namespace SpacePartitioning { namespace details {
//	template<>
//	class BVHObjectTraits<Triangle> {
//	public:
//		static bool intersect( const Triangle &t, const Ray& r, float* dist ) { return r.calcTriangleIntersection( t.a, t.b, t.c, dist ); }
//	};
//} }

class RayCastingApp : public App {
public:
	void setup() override;
	void draw() override;

	CameraPersp			mCamera;
	CameraUi			mCameraUi;

	gl::BatchRef		mTeapot;
	
	vector<Triangle>	mTriangles;
	sp::BVH<Triangle>	mBvh;
};

void RayCastingApp::setup()
{
	// setup the camera and ui
	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 50.0f, 0.01f, 10.0f );
	mCameraUi = CameraUi( &mCamera, getWindow() );
	mCamera.lookAt( vec3( 1.0f ), vec3( 0.0f ) );

	// create a teapot with roughly a million triangles
	TriMesh mesh = geom::Teapot().subdivisions( 128 );
	mTeapot = gl::Batch::create( mesh, gl::getStockShader( gl::ShaderDef().color().lambert() ) );

	// store the triangles in a vector so we can use them with the bounding volume hierarchy
	for( size_t i = 0; i < mesh.getNumTriangles(); ++i ) {
		Triangle t;
		mesh.getTriangleVertices( i, &t.a, &t.b, &t.c );
		mTriangles.push_back( t );
	}

	// create the bounding volume hierarchy
	mBvh =	sp::BVH<Triangle>( &mTriangles );
}

void RayCastingApp::draw()
{
	// clear the screen and set the matrics
	gl::clear( Color::gray( 0.6f ) ); 
	gl::setMatrices( mCamera );

	// render the teapot
	gl::ScopedDepth scopedDepth( true );
	mTeapot->draw();

	// cast a ray from the mouse position
	auto mousePos = clamp( vec2( getMousePos() - getWindowPos() ), vec2( 0.0f ), vec2( getWindowSize() ) );
	auto ray = mCamera.generateRay( mousePos, getWindowSize() );
	sp::BVH<Triangle>::RaycastResult result;
	// if there's a hit draw the triangle at the intersection
	if( mBvh.raycast( ray, &result ) ) {
		gl::ScopedDepth scopedDepth( false );
		gl::ScopedColor scopedColor( ColorA::black() );
		gl::VertBatch triangle;
		triangle.begin( GL_LINE_LOOP );
		triangle.vertex( result.getObject()->a );
		triangle.vertex( result.getObject()->b );
		triangle.vertex( result.getObject()->c );
		triangle.end();
		triangle.draw();
	}
}

CINDER_APP( RayCastingApp, RendererGl( RendererGl::Options().msaa( 8 ) ) )
