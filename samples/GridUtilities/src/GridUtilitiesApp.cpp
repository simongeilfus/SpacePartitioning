#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

#include "sp/Grid.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class GridUtilitiesApp : public App {
  public:
	void setup() override;
	void draw() override;
	
	gl::BatchRef	mSamples;
	sp::Grid2<>		mGrid;
};

void GridUtilitiesApp::setup()
{
	// initialize the grid
	mGrid = sp::Grid2<>( 5 );
	
	// create our data set with some random points in the center of the screen
	randSeed( 1234567 );
	vector<vec2> samples;
	for( int i = 0; i < 100; i++ ){
		auto randPos = getWindowCenter() + randVec2() * randFloat( 0.0f, 100.0f );
		mGrid.insert( randPos );
		samples.push_back( randPos );
	}
	
	// create a batch to draw the samples
	auto layout = geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 2, 0, 0 ) } );
	auto vboMesh = gl::VboMesh::create( samples.size(), GL_POINTS, { { layout, gl::Vbo::create( GL_ARRAY_BUFFER, samples ) } } );
	mSamples = gl::Batch::create( vboMesh, gl::getStockShader( gl::ShaderDef().color() ) );
	
	hideCursor();
}

void GridUtilitiesApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	// draw the grid bounds
	gl::ScopedColor color0( ColorA::gray( 1.0f, 0.15f ) );
	auto gridBounds = mGrid.getBounds();
	gl::drawStrokedRect( gridBounds );
	
	// draw the samples found in a 10 pixels radius around the mouse
	auto neighbors = mGrid.rangeSearch( getMousePos() - getWindowPos(), 10.0f );
	gl::drawStrokedCircle( getMousePos() - getWindowPos(), 10.0f );
	gl::ScopedColor red( ColorA( 1.0f, 0.0f, 0.0f, 1.0f ) );
	for( auto n : neighbors ) {
		gl::drawStrokedCircle( n.first->getPosition(), 4.0f );
	}
	
	// draw the samples
	gl::ScopedColor color1( ColorA::gray( 1.0f, 1.0f ) );
	mSamples->draw();
	
	// get all the grid cells and draw the ones that have samples
	size_t bins = mGrid.getNumBins();
	for( size_t i = 0; i < bins; ++i ) {
		auto bin = mGrid.getBin( i );
		if( bin.size() ) {
			gl::drawStrokedRect( mGrid.getBinBounds( i ) );
		}
	}
	
	// try to get a bin at the mouse position
	try {
		auto binIndex = mGrid.getBinIndexAt( getMousePos() - getWindowPos() );
		gl::ScopedColor red( ColorA( 1.0f, 0.0f, 0.0f, 1.0f ) );
		gl::drawStrokedRect( mGrid.getBinBounds( binIndex ) );
		gl::drawStringCentered( to_string( mGrid.getBin( binIndex ).size() ), mGrid.getBinCenter( binIndex ), Color(1,0,0) );
		gl::drawString( "Bin #" + to_string( binIndex ), gridBounds.getUpperLeft() - vec2( 0, 10 ) );
	}
	// the mouse position might be outside of the grid bounds
	catch( const sp::GridOutOfBoundsException &exc ) {}
}

CINDER_APP( GridUtilitiesApp, RendererGl )
