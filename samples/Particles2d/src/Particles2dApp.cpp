#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

#include "SpacePartitioning.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct Particle {
	void operator+=( const vec2 &position ){ mPosition += position; }
	
	vec2 mPosition, mLastPosition;
};

class Particles2dApp : public App {
  public:
	Particles2dApp();
	void update() override;
	void draw() override;
	void simulate( double timestep );
	void mouseDrag( MouseEvent event ) override;
	
	sp::Grid2		mParticleGrid;
	gl::BatchRef		mParticlesBatch;
	gl::VboMeshRef		mParticlesVbo;
	vector<Particle>	mParticles;
};

Particles2dApp::Particles2dApp()
{
	// init the particle positions
	for( size_t i = 0; i < 12500; ++i ) {
		auto pos = vec2( randFloat(), randFloat() ) * vec2( getWindowSize() );
		mParticles.push_back( { pos, pos + randVec2() } );
	}
	
	// create a vbo and a batch to render the particles
	mParticlesVbo = gl::VboMesh::create( mParticles.size(), GL_POINTS, {
		{
			geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 2, 0, 0, 0 ) } ),
			gl::Vbo::create( GL_ARRAY_BUFFER, mParticles, GL_DYNAMIC_DRAW )
		}
	} );
	mParticlesBatch = gl::Batch::create( mParticlesVbo, gl::getStockShader( gl::ShaderDef().color() ) );
	
	// and the grid that will be user to do a really basic n-body simulation
	mParticleGrid = sp::Grid2( vec2( 0 ), vec2( getWindowSize() ), 2 );
}

void Particles2dApp::update()
{
	// update the grid
	mParticleGrid.clear();
	for( auto& particle : mParticles ) {
		mParticleGrid.insert( particle.mPosition, static_cast<void*>( &particle ) );
	}
	
	// update particle-particle forces
	for( auto& particle : mParticles ) {
		// find nearest neighbors and apply repulsion forces
		mParticleGrid.rangeSearch( particle.mPosition, 7.5f, [&particle]( sp::Grid2::Node* neighbor, float distSq ) {
			// check the distance square to see if it's the same particle or a neighbor
			if( distSq > 0 ) {
				auto neighborParticle = static_cast<Particle*>( neighbor->getData() );
				auto diff = ( particle.mPosition - neighborParticle->mPosition ) * 0.025f;
				particle += diff;
				*neighborParticle += -diff;
			}
		} );
	}
	
	// physic integration steps
	static const double timestep = 1.0 / 60.0;
	static double time = getElapsedSeconds();
	static double accumulator = 0.0;
	
	double elapsed	= getElapsedSeconds() - time;
	time		+= elapsed;
	accumulator	+= glm::min( elapsed, 0.1 );
	while( accumulator >= timestep ) {
		simulate( timestep );
		accumulator -= timestep;
	}
	
	// update particle vbo
	gl::VboMesh::MappedAttrib<vec2> mappedPos = mParticlesVbo->mapAttrib2f( geom::POSITION );
	for( size_t i = 0; i < mParticlesVbo->getNumVertices(); i++ ) {
		*mappedPos = mParticles[i].mPosition;
		mappedPos++;
	}
	mappedPos.unmap();
	
	getWindow()->setTitle( to_string( (int) getAverageFps() ) );
}
void Particles2dApp::simulate( double timestep )
{
	// update particles positions
	for( auto& particle : mParticles ) {
		// constraint to windows bounds
		particle.mPosition = glm::max( glm::min( particle.mPosition, vec2( getWindowSize() ) ), vec2( 0 ) );
		
		// basic verlet integration
		const float friction = 0.985f;
		auto temp = particle.mPosition;
		particle += ( particle.mPosition - particle.mLastPosition ) * friction;
		particle.mLastPosition = temp;
	}
}
void Particles2dApp::mouseDrag( MouseEvent event )
{
	// get the particles around the mouse
	vec2 mouse = event.getPos();
	mParticleGrid.rangeSearch( mouse, event.isLeft() ? 100.0f : 25.0f, [mouse,event]( sp::Grid2::Node* node, float distSq ) {
		// and move them away
		auto particle = static_cast<Particle*>( node->getData() );
		*particle += ( particle->mPosition - mouse ) * ( event.isLeft() ? 0.005f : 0.5f );

	} );
}

void Particles2dApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	mParticlesBatch->draw();
}

CINDER_APP( Particles2dApp, RendererGl, []( App::Settings* settings ) {
 settings->setWindowSize( 1280, 720 );
})
