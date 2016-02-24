#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CameraUi.h"
#include "cinder/Rand.h"
#include "cinder/Perlin.h"

#include "SpacePartitioning.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// Minimal verlet particle class
struct Particle {
	void operator+=( const vec3 &position ){ mPosition += position; }
	vec3 mPosition, mLastPosition;
	float mSize, mId;
};

class Particles2dApp : public App {
  public:
	Particles2dApp();
	void update() override;
	void draw() override;
	void simulate( double timestep );
	void mouseDrag( MouseEvent event ) override;
	
	CameraPersp		mCamera;
	CameraUi		mCameraUi;
	
	sp::Grid3		mParticleGrid;
	gl::BatchRef		mParticlesBatch;
	gl::VboMeshRef		mParticlesVbo;
	vector<Particle>	mParticles;
	size_t			mConnections;
	gl::BatchRef		mConnectionsBatch;
	gl::VboMeshRef		mConnectionsVbo;
};

Particles2dApp::Particles2dApp()
{
	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 60, 0.1, 5000 ).calcFraming( Sphere( vec3(0), 50.0f ) );
	mCameraUi = CameraUi( &mCamera, getWindow() );
	
	// init the particle positions
	vector<vec3> vertices;
	vector<float> custom0;
	for( size_t i = 0; i < 8000; ++i ) {
		auto pos = randVec3() * randFloat( 50.0f, 80.0f );
		auto size = 1.0f;//randFloat( 0.4f, 2.0f );
		mParticles.push_back( { pos, pos + randVec3(), size, static_cast<float>( i ) } );
		vertices.push_back( pos );
		custom0.push_back( size );
	}
	
	// create a vbo and a batch to render the particles
	mParticlesVbo = gl::VboMesh::create( vertices.size(), GL_POINTS, {
		{
			geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 3, 0, 0, 0 ) } ),
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
										 .attrib( geom::CUSTOM_0, "ciCustom0" )
										 ) );
	
	
	
	// create an empty vbo for the particles connections
	vector<vec3> connections( 1000000, vec3(0) );
	mConnectionsVbo = gl::VboMesh::create( connections.size(), GL_LINES, {
		{
			geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 3, 0, 0, 0 ) } ),
			gl::Vbo::create( GL_ARRAY_BUFFER, connections, GL_DYNAMIC_DRAW )
		}
	} );
	mConnectionsBatch = gl::Batch::create( mConnectionsVbo, gl::getStockShader( gl::ShaderDef().color() ) );
	
	
	// and the grid that will be used to do a really basic n-body simulation
	mParticleGrid = sp::Grid3( vec3( -100 ), vec3( 100 ), 5 );
}

void Particles2dApp::update()
{
	// update the grid
	mParticleGrid.clear();
	for( auto& particle : mParticles ) {
		mParticleGrid.insert( particle.mPosition, static_cast<void*>( &particle ) );
	}
	
	int notConnected = 0;
	mConnections = 0;
	static Perlin perlin;
	gl::VboMesh::MappedAttrib<vec3> mappedPos0 = mConnectionsVbo->mapAttrib3f( geom::POSITION );
	// update particle-particle forces
	for( auto& particle : mParticles ) {
		// find nearest neighbors and apply repulsion forces
		float noisyRadius = 4.0f;//glm::abs( perlin.fBm( particle.mPosition * 0.01f ) ) * particle.mSize * 16.0f;
		mParticleGrid.rangeSearch( particle.mPosition, noisyRadius, [this, &mappedPos0, &particle, &notConnected]( sp::Grid3::Node* neighbor, float distSq ) {
			// check the distance square to see if it's the same particle or a neighbor
			auto neighborParticle = static_cast<Particle*>( neighbor->getData() );
			if( neighborParticle != &particle ) {
				auto diff = ( particle.mPosition - neighborParticle->mPosition );
				if( mConnections < mConnectionsVbo->getNumVertices() - 1 ) {
					*mappedPos0 = particle.mPosition;
					mappedPos0++;
					*mappedPos0 = neighborParticle->mPosition;
					mappedPos0++;
					mConnections++;
				}
				else notConnected++;
				float radSq = 3;
				if( distSq < 1.0f ) {
					particle += perlin.dfBm( particle.mPosition * 0.075f ) * 0.125f;
					*neighborParticle += -perlin.dfBm( neighborParticle->mPosition * 0.075f ) * 0.125f;
				}
				if( distSq < radSq ) {
					float invDist = ( radSq - distSq );
					particle += diff * 0.025f * invDist;
					*neighborParticle += -diff * 0.025f * invDist;
				}
			}
		} );
		
		const float time = getElapsedSeconds() * 0.05f;
		particle += perlin.dfBm( particle.mPosition * 0.01f + vec3( time + particle.mId * 0.000f ) ) * 0.05f * glm::max( 0.0f, perlin.fBm( -particle.mPosition * 0.005f - vec3( time * 5.0f ) ) );
		
		float noiseSphere = 50.0f + perlin.fBm( particle.mPosition * 0.01f + vec3( getElapsedSeconds() * 0.05f ) ) * 90.0f * glm::max( 0.0f, perlin.fBm( -particle.mPosition * 0.05f - vec3( getElapsedSeconds() * 0.01f ) ) );
		if( length2( particle.mPosition ) < noiseSphere * noiseSphere ) {
			// sphere constraint
			particle.mPosition = normalize( particle.mPosition ) * noiseSphere;
		}
		else {
			// apply weak force that attracts all particle to the center
			particle += ( vec3( 0 ) - particle.mPosition ) * 0.00025f * particle.mSize;
		}
	}
	mappedPos0.unmap();
	
	if( notConnected ) cout << mConnections << "/" << mConnectionsVbo->getNumVertices() << " " << notConnected << endl;
	
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
	gl::VboMesh::MappedAttrib<vec3> mappedPos1 = mParticlesVbo->mapAttrib3f( geom::POSITION );
	for( size_t i = 0; i < mParticlesVbo->getNumVertices(); i++ ) {
		*mappedPos1 = mParticles[i].mPosition;
		mappedPos1++;
	}
	mappedPos1.unmap();
	
	getWindow()->setTitle( to_string( (int) getAverageFps() ) );
}
void Particles2dApp::simulate( double timestep )
{
	// update particles positions
	for( auto& particle : mParticles ) {
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
/*	mParticleGrid.rangeSearch( mouse, event.isLeft() ? 100.0f : 25.0f, [mouse,event]( sp::Grid3::Node* node, float distSq ) {
		// and move them away
		auto particle = static_cast<Particle*>( node->getData() );
		*particle += ( particle->mPosition - mouse ) * ( event.isLeft() ? 0.005f : 0.5f );

	} );*/
}

void Particles2dApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	gl::setMatrices( mCamera );
	/*{
		gl::ScopedDepth depthReadAndWrite( true );
		gl::ScopedColor color0( ColorA::gray( 0.0f ) );
		gl::drawSphere( Sphere( vec3( 0.0f ), 49.5f ) );
	}
	
	gl::ScopedDepthTest depthTestOnly(true);*/
	//gl::ScopedBlendAlpha alphaBlending;
	gl::ScopedBlendAdditive additiveBlending;
	gl::ScopedState glslPointSize( GL_PROGRAM_POINT_SIZE, true );
	gl::ScopedColor color1( ColorA::gray( 1.0f, 0.05f ) );
	mParticlesBatch->draw();
	mConnectionsBatch->draw( 0, mConnections * 2 );
}

CINDER_APP( Particles2dApp, RendererGl, []( App::Settings* settings ) {
 settings->setWindowSize( 1280, 720 );
})
