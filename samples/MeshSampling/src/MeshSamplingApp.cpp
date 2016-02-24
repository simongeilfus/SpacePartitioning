#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/TriMesh.h"
#include "cinder/CameraUi.h"
#include "cinder/Rand.h"
#include "cinder/Timer.h"

#include "sp/Grid.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class MeshSamplingApp : public App {
public:
	void setup() override;
	void draw() override;
	
	TriMeshRef		mModelMesh;
	gl::BatchRef	mModel, mSamples;
	CameraPersp		mCamera;
	CameraUi		mCameraUi;
};

void MeshSamplingApp::setup()
{
	mModelMesh = TriMesh::create( geom::Teapot() );
	
	Timer timer( true );
	
	// Poisson disk parameters
	size_t k = 15; // the higher k is, the tighter the packing will be but the slower the algorithm
	float separation = 0.015f;	// this is the min separation between samples
	
	// Grid Parameters ( tweak this in relation to the mesh size )
	float gridScale = 300.0f; // if the object is too small, the sp::Grid will not improve anything
	
	// create two grid from the size of the mesh
	// one for the triangles and another for checking distance between samples
	auto boundingBox = mModelMesh->calcBoundingBox();
	auto grid = sp::Grid3<int>( gridScale * boundingBox.getMin(), gridScale * boundingBox.getMax() );
	auto meshGrid = sp::Grid3<int>( gridScale * boundingBox.getMin(), gridScale * boundingBox.getMax() );
	
	// start by putting each vertices of the mesh in the structure
	// storing a reference to the triangle
	auto triangles = mModelMesh->getNumTriangles();
	for( size_t i = 0; i < triangles; i++ ) {
		vec3 a, b, c;
		mModelMesh->getTriangleVertices( i, &a, &b, &c );
		meshGrid.insert( gridScale * a, i );
		meshGrid.insert( gridScale * b, i );
		meshGrid.insert( gridScale * c, i );
	}
	
	// we have two lists, one with the samples that need to be checked
	// and another with the accepted samples.
	vector<vec3> processingList;
	vector<vec3> samplesList;
	
	// pick a few random starting point from the mesh and add them to the processingList
	int startingPoints = glm::max( 1, (int) mModelMesh->getNumVertices() / 100 );
	auto vertices = mModelMesh->getPositions<3>();
	for( size_t i = 0; i < startingPoints; i++ ) {
		auto p = vertices[randInt( 0, mModelMesh->getNumVertices() - 1 )];
		processingList.push_back( p );
		samplesList.push_back( p );
		grid.insert( gridScale * p );
	}
	
	// while there's points in the processing list
	while( processingList.size() ){
		
		// pick a random point in the processing list and remove it from the list
		int randPoint = randInt( 0, processingList.size() - 1 );
		vec3 center = processingList[randPoint];
		processingList.erase( processingList.begin() + randPoint );
		
		// ideally we'd want to spawn k points in an sphere around that point
		// here instead we'll get the mesh triangles around the point, spawn
		// k random point on each triangles and choose in the k points the ones
		// that doesn't have neighbors that are closer than the min sep distance
		meshGrid.rangeSearch( gridScale * center, gridScale * separation * 2.0f, [&]( sp::Grid3<int>::Node *node, float distance ) {
			vec3 a, b, c;
			int triangleId = node->getData();
			mModelMesh->getTriangleVertices( triangleId, &a, &b, &c );
			vec3 ab = b - a, ac = c - a;
			
			// get k random point on the current triangle
			for( size_t i = 0; i < k; i++ ){
				vec2 uv = vec2( randFloat(), randFloat() );
				if( uv.x + uv.y > 1.0f ) uv = vec2( 1.0f ) - uv;
				vec3 newPoint = a + ab * uv.x  + ac * uv.y;
				
				// check if the new point has no neighbors that are too close to it
				// = no neighbor in the "separation" range
				if( !grid.rangeSearch( gridScale * newPoint, gridScale * separation ).size() ){
					
					// if the point is selected add it to the processing list, output list and grid
					processingList.push_back( newPoint );
					samplesList.push_back( newPoint );
					grid.insert( gridScale * newPoint );
				}
			}
		} );
	}
	
	cout << "Took " << timer.getSeconds() * 1000.0 << "ms to generate " << samplesList.size() << " samples" << endl;
	
	// rendering
	mModelMesh->recalculateNormals();
	mModel = gl::Batch::create( *mModelMesh, gl::getStockShader( gl::ShaderDef().color().lambert() ) );
	auto layout = geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 3, 0, 0 ) } );
	auto vboMesh = gl::VboMesh::create( samplesList.size(), GL_POINTS, { { layout, gl::Vbo::create( GL_ARRAY_BUFFER, samplesList ) } } );
	mSamples = gl::Batch::create( vboMesh, gl::getStockShader( gl::ShaderDef().color() ) );
	
	mCamera	= CameraPersp( getWindowWidth(), getWindowHeight(), 60, 0.1, 1000 ).calcFraming( Sphere( vec3(0), 1 ) );
	mCameraUi = CameraUi( &mCamera, getWindow(), -1 );
}


void MeshSamplingApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	gl::setMatrices( mCamera );
	gl::ScopedDepth enableDepth( true );
	
	gl::ScopedColor white( ColorA::white() );
	mModel->draw();
	
	gl::ScopedColor red( ColorA( 1.0f, 0.0f, 0.0f, 1.0f ) );
	gl::pointSize( 2.5f );
	//mSamples->draw( 0, ( getElapsedFrames() * 20 ) % mSamples->getNumVertices() );
	mSamples->draw();
}

CINDER_APP( MeshSamplingApp, RendererGl )