#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/TriMesh.h"
#include "cinder/CameraUi.h"
#include "cinder/Rand.h"

#include "sp/Grid.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class MeshVoxelsApp : public App {
  public:
	void setup() override;
	void draw() override;
	
	CameraPersp		mCamera;
	CameraUi		mCameraUi;
	gl::BatchRef	mVoxels, mVoxelsLines;
	size_t			mNumVoxels;
};

void MeshVoxelsApp::setup()
{
	// create the mesh
	auto mesh = TriMesh::create( geom::Teapot() );
	
	// create a grid of the size of the mesh
	float gridScale = 300.0f;
	auto boundingBox = mesh->calcBoundingBox();
	auto meshGrid = sp::Grid3<>( gridScale * boundingBox.getMin(), gridScale * boundingBox.getMax(), 2 );
	
	// start by adding random points on each triangles of the mesh in the structure
	auto triangles = mesh->getNumTriangles();
	for( size_t i = 0; i < triangles; i++ ) {
		vec3 a, b, c;
		mesh->getTriangleVertices( i, &a, &b, &c );
		vec3 ab = b - a, ac = c - a;
		// add enough random point per triangle
		for( size_t i = 0; i < 300; i++ ){
			vec2 uv = vec2( randFloat(), randFloat() );
			if( uv.x + uv.y > 1.0f ) uv = vec2( 1.0f ) - uv;
			vec3 p = a + ab * uv.x  + ac * uv.y;
			meshGrid.insert( gridScale * p );
		}
		// add the triangle vertices just to make sure
		meshGrid.insert( gridScale * a );
		meshGrid.insert( gridScale * b );
		meshGrid.insert( gridScale * c );
	}
	
	// we'll use the position of each non-empty bin to create our voxels
	// we could iterate over each potential voxels in the mesh bounds and use the
	// grid to see if there's triangles in each of them, but this is a bit easier
	vector<vec3> positions;
	size_t bins = meshGrid.getNumBins();
	for( size_t i = 0; i < bins; ++i ) {
		auto bin = meshGrid.getBin( i );
		if( bin.size() ) {
			positions.push_back( meshGrid.getBinCenter( i ) / gridScale );
		}
	}
	mNumVoxels = positions.size();
	
	// create the instanced batch
	auto vboMesh = gl::VboMesh::create( geom::Cube().size( meshGrid.getBinsSize() / gridScale ) );
	auto vboMeshLines = gl::VboMesh::create( geom::WireCube().size( meshGrid.getBinsSize() / gridScale ) );
	auto layout = geom::BufferLayout( { geom::AttribInfo( geom::CUSTOM_0, 3, 0, 0, 1 ) } );
	vboMesh->appendVbo( layout, gl::Vbo::create( GL_ARRAY_BUFFER, positions ) );
	vboMeshLines->appendVbo( layout, gl::Vbo::create( GL_ARRAY_BUFFER, positions ) );
	auto glslLambert = gl::GlslProg::create( loadAsset( "lambert.vert" ), loadAsset( "lambert.frag" ) );
	auto glslColor = gl::GlslProg::create( loadAsset( "color.vert" ), loadAsset( "color.frag" ) );
	mVoxels = gl::Batch::create( vboMesh, glslLambert, { { geom::CUSTOM_0, "vInstancePosition" } } );
	mVoxelsLines = gl::Batch::create( vboMeshLines, glslColor, { { geom::CUSTOM_0, "vInstancePosition" } } );
	
	// setup camera
	mCamera	= CameraPersp( getWindowWidth(), getWindowHeight(), 60, 0.1, 1000 );
	mCamera.lookAt( vec3( -0.699,    1.103,    1.004 ), vec3( 0.0f ) );
	mCamera.setOrientation( quat( 0.931,   -0.273,   -0.233,   -0.068 ) );
	mCameraUi = CameraUi( &mCamera, getWindow(), -1 );
}

void MeshVoxelsApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::setMatrices( mCamera );
	gl::ScopedDepth enableDepth( true );
	mVoxels->drawInstanced( mNumVoxels );
	mVoxelsLines->drawInstanced( mNumVoxels );
}

CINDER_APP( MeshVoxelsApp, RendererGl( RendererGl::Options().msaa( 8 ) ) )
