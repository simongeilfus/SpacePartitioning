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
	Triangle( const vec3 &a, const vec3 &b, const vec3 &c ) : mVertices( { a, b, c } ), mBounds( min( a, min( b, c ) ), max( a, max( b, c ) ) ), mCentroid( ( a + b + c ) / 3.0f ) {}
	ci::AxisAlignedBox getBounds() const { return mBounds; }
	vec3 getCentroid() const { return mCentroid; }
	bool intersect( const Ray& r, float* dist ) const { return r.calcTriangleIntersection( mVertices[0], mVertices[1], mVertices[2], dist ); }
	vec3 getA() const { return mVertices[0]; }
	vec3 getB() const { return mVertices[1]; }
	vec3 getC() const { return mVertices[2]; }
protected:
	AxisAlignedBox mBounds;
	vec3 mCentroid;
	array<vec3,3> mVertices;
};

// If the class doesn't provide those functions you can specialize BVHObjectTraits to specify your own
//namespace SpacePartitioning { namespace details {
//	template<>
//	class BVHObjectTraits<Triangle> {
//	public:
//		static bool intersect( const Triangle &t, const Ray& r, float* dist ) { return r.calcTriangleIntersection( t.getA(), t.getB(), t.getC(), dist ); }
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
		vec3 a, b, c;
		mesh.getTriangleVertices( i, &a, &b, &c );
		mTriangles.push_back( Triangle( a, b, c ) );
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
	
	Timer timer( true );
	bool hit = mBvh.raycast( ray, &result );
	timer.stop();

	// if there's a hit draw the triangle at the intersection
	if( hit ) {
		gl::ScopedDepth scopedDepth( false );
		gl::ScopedColor scopedColor( ColorA::black() );
		gl::VertBatch triangle;
		triangle.begin( GL_LINE_LOOP );
		triangle.vertex( result.getObject()->getA() );
		triangle.vertex( result.getObject()->getB() );
		triangle.vertex( result.getObject()->getC() );
		triangle.end();
		triangle.draw();
	}

	// get timing avg on 30 frames
	static int framesAvg = 0;
	static float timeAvg = 0.0f, timeAcc = 0.0f;
	if( ++framesAvg > 30 ) {
		timeAvg = timeAcc / 30.0f;
		timeAcc = framesAvg = 0;
	}
	timeAcc += timer.getSeconds(); 
	getWindow()->setTitle( "RayCasting " + to_string( timeAvg * 1000.0 ) + "ms " );
}

CINDER_APP( RayCastingApp, RendererGl( RendererGl::Options().msaa( 8 ) ) )
