#include "HashTable.h"
#include "Grid.h"
#include "KdTree.h"

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// Minimal verlet particle class
struct Particle {
	void operator+=( const vec2 &position ){ mPosition += position; }
	vec2 mPosition, mLastPosition;
	float mSize;
};

class Particles2dApp : public App {
  public:
	Particles2dApp();
	void update() override;
	void draw() override;
	void simulate( double timestep );
	void mouseDrag( MouseEvent event ) override;
	
	//using SpatialStruct = sp::KdTree2_t<Particle*>;
	using SpatialStruct = sp::Grid2_t<Particle*>;
	
	SpatialStruct		mParticleSpatialStruct;
	gl::BatchRef		mParticlesBatch;
	gl::VboMeshRef		mParticlesVbo;
	vector<Particle>	mParticles;
};

Particles2dApp::Particles2dApp()
{
	// init the particle positions
	vector<vec2> vertices;
	vector<float> custom0;
	for( size_t i = 0; i < 20000; ++i ) {
		auto pos = vec2( randFloat(), randFloat() ) * vec2( getWindowSize() );
		auto size = randFloat( 0.4f, 2.0f );
		if( randFloat() < 0.015 )
			size = randFloat( 2.0f, 6.0f );
		mParticles.push_back( { pos, pos + randVec2(), size } );
		vertices.push_back( pos );
		custom0.push_back( size );
	}
	
	// create a vbo and a batch to render the particles
	mParticlesVbo = gl::VboMesh::create( vertices.size(), GL_POINTS, {
		{
			geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 2, 0, 0, 0 ) } ),
			gl::Vbo::create( GL_ARRAY_BUFFER, vertices, GL_DYNAMIC_DRAW )
		},
		{
			geom::BufferLayout( { geom::AttribInfo( geom::CUSTOM_0, 1, 0, 0, 0 ) } ),
			gl::Vbo::create( GL_ARRAY_BUFFER, custom0, GL_STATIC_DRAW )
		}
	} );
	mParticlesBatch = gl::Batch::create( mParticlesVbo, gl::GlslProg::create( gl::GlslProg::Format()
										 .vertex( loadAsset( "shader.vert" ) )
										 .fragment( loadAsset( "shader.frag" ) )
										 .attrib( geom::POSITION, "ciPosition" )
										 .attrib( geom::CUSTOM_0, "ciCustom0" ) ) );
	
	// and the grid that will be user to do a really basic n-body simulation
	mParticleSpatialStruct = SpatialStruct();
}

void Particles2dApp::update()
{
	// update the grid
	mParticleSpatialStruct.clear();
	for( auto& particle : mParticles ) {
		mParticleSpatialStruct.insert( particle.mPosition, &particle );
	}
	
	// update particle-particle forces
	for( auto& particle : mParticles ) {
		// find nearest neighbors and apply repulsion forces
		mParticleSpatialStruct.rangeSearch( particle.mPosition, particle.mSize * 3.0f, [&particle]( SpatialStruct::Node* neighbor, float distSq ) {
			// check the distance square to see if it's the same particle or a neighbor
			if( distSq > 0 ) {
				auto neighborParticle = neighbor->getData();
				auto diff = ( particle.mPosition - neighborParticle->mPosition );
				particle += diff * 0.0075f;
				*neighborParticle += -diff * 0.0075f;
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
		
		// apply weak force that attracts all particle to the center
		particle += ( vec2( getWindowCenter() ) - particle.mPosition ) * 0.000001f;
		
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
	mParticleSpatialStruct.rangeSearch( mouse, event.isLeft() ? 100.0f : 25.0f, [mouse,event]( SpatialStruct::Node* node, float distSq ) {
		// and move them away
		auto particle = static_cast<Particle*>( node->getData() );
		*particle += ( particle->mPosition - mouse ) * ( event.isLeft() ? 0.005f : 0.5f );

	} );
}

void Particles2dApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	gl::ScopedBlendAlpha alphaBlending;
	gl::ScopedState glslPointSize( GL_PROGRAM_POINT_SIZE, true );
	mParticlesBatch->draw();
}

CINDER_APP( Particles2dApp, RendererGl, []( App::Settings* settings ) {
 settings->setWindowSize( 1280, 720 );
})
