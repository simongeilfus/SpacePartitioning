#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

#include "sp/Grid.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define MAX_SAMPLES 1000000

class PoissonDiskDistributionApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
	
	sp::Grid2<>		mGrid;
	vector<vec2>	mProcessingList;
	vector<vec2>	mSamples;
	gl::BatchRef	mCircles;
};

void PoissonDiskDistributionApp::setup()
{
	// add a point to the processing list, the samples and the grid
	auto start = getWindowCenter();
	mProcessingList.push_back( start );
	mSamples.push_back( start );
	mGrid.insert( start );
	
	// setup rendering
	auto layout = geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 2, 0, 0 ) } );
	auto vboMesh = gl::VboMesh::create( MAX_SAMPLES, GL_POINTS, { { layout, gl::Vbo::create( GL_ARRAY_BUFFER, MAX_SAMPLES * sizeof(vec2), nullptr, GL_DYNAMIC_DRAW ) } } );
	auto glsl = gl::GlslProg::create( loadAsset( "shader.vert" ), loadAsset( "shader.frag" ) );
	mCircles = gl::Batch::create( vboMesh, glsl );
}

void PoissonDiskDistributionApp::mouseDown( MouseEvent event )
{
	// restart the whole process
	mProcessingList.clear();
	mSamples.clear();
	mGrid.clear();
	mProcessingList.push_back( event.getPos() );
	mSamples.push_back( event.getPos() );
	mGrid.insert( event.getPos() );
}

void PoissonDiskDistributionApp::update()
{
	// if the processing list is not empty process the next points
	if( mProcessingList.size() ) {
		size_t start = mSamples.size();
		size_t pointToProcess = glm::max( 10, (int) mProcessingList.size() / 10 );
		for( size_t j = 0; j < pointToProcess && mProcessingList.size(); ++j ) {
			// pick a random point in the processing list
			int randPoint = randInt( 0, mProcessingList.size() - 1 );
			vec2 center = mProcessingList[randPoint];
			
			// remove it
			mProcessingList.erase( mProcessingList.begin() + randPoint );
			
			// spawn k points in an anulus around that point
			// the higher k is, the higher the packing will be and slower the algorithm
			int k = 35;
			float separation = 5.0f;
			for( int i = 0; i < k; i++ ){
				float randRadius	= separation * ( 1.0f + randFloat() );
				float randAngle		= randFloat() * M_PI * 2.0f;
				vec2 newPoint		= center + vec2( cos( randAngle ), sin( randAngle ) ) * randRadius;
				
				// check if the new random point is in the window bounds
				// and if it has no neighbors that are too close to them
				if( getWindowBounds().contains( newPoint )
				   && !mGrid.rangeSearch( newPoint, separation ).size() ){
					
					// if the point has no close neighbors add it to the processing list, output list and grid
					mProcessingList.push_back( newPoint );
					mSamples.push_back( newPoint );
					mGrid.insert( newPoint );
				}
			}
		}
		
		// update the vbo mesh
		gl::VboMesh::MappedAttrib<vec2> positions = mCircles->getVboMesh()->mapAttrib2f( geom::Attrib::POSITION );
		for( size_t i = start; i < mSamples.size() && i < MAX_SAMPLES; ++i ) {
			positions[i] = mSamples[i];
		}
		positions.unmap();
	}
	
	// Display some infos in the window bar
	getWindow()->setTitle( "Fps: " + to_string( (int) getAverageFps() ) + " | ProcessingList size: " + to_string( mProcessingList.size() ) );
}

void PoissonDiskDistributionApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	// draw the samples
	gl::pointSize( 4.0f );
	mCircles->draw( 0, mSamples.size() );
}

CINDER_APP( PoissonDiskDistributionApp, RendererGl )
