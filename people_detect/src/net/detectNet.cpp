/*
 * http://github.com/dusty-nv/jetson-inference
 */
 
#include "detectNet.h"

#include "cudaMappedMemory.h"
#include "cudaOverlay.h"
#include "cudaResize.h"

#include "commandLine.h"

#define OUTPUT_CVG  0
#define OUTPUT_BBOX 1

//#define DEBUG_CLUSTERING


// constructor
detectNet::detectNet() : tensorNet()
{
	mCoverageThreshold = 0.5f;
	
	mClassColors[0] = NULL;	// cpu ptr
	mClassColors[1] = NULL; // gpu ptr
}


// destructor
detectNet::~detectNet()
{
	
}


// Create
detectNet* detectNet::Create( const char* prototxt, const char* model, const char* mean_binary, float threshold, const char* input_blob, const char* coverage_blob, const char* bbox_blob, uint32_t maxBatchSize )
{
	detectNet* net = new detectNet();
	
	if( !net )
		return NULL;

	printf("\n");
	printf("detectNet -- loading detection network model from:\n");
	printf("          -- prototxt    %s\n", prototxt);
	printf("          -- model       %s\n", model);
	printf("          -- input_blob  '%s'\n", input_blob);
	printf("          -- output_cvg  '%s'\n", coverage_blob);
	printf("          -- output_bbox '%s'\n", bbox_blob);
	printf("          -- threshold   %f\n", threshold);
	printf("          -- batch_size  %u\n\n", maxBatchSize);
	
	//net->EnableDebug();
	
	std::vector<std::string> output_blobs;
	output_blobs.push_back(coverage_blob);
	output_blobs.push_back(bbox_blob);
	
	if( !net->LoadNetwork(prototxt, model, mean_binary, input_blob, output_blobs) )
	{
		printf("detectNet -- failed to initialize.\n");
		return NULL;
	}
	
	const uint32_t numClasses = net->GetNumClasses();
	
	if( !cudaAllocMapped((void**)&net->mClassColors[0], (void**)&net->mClassColors[1], numClasses * sizeof(float4)) )
		return NULL;
	
	for( uint32_t n=0; n < numClasses; n++ )
	{
		if( n != 1 )
		{
			net->mClassColors[0][n*4+0] = 0.0f;	// r
			net->mClassColors[0][n*4+1] = 200.0f;	// g
			net->mClassColors[0][n*4+2] = 255.0f;	// b
			net->mClassColors[0][n*4+3] = 100.0f;	// a
		}
		else
		{
			net->mClassColors[0][n*4+0] = 0.0f;	// r
			net->mClassColors[0][n*4+1] = 255.0f;	// g
			net->mClassColors[0][n*4+2] = 175.0f;	// b
			net->mClassColors[0][n*4+3] = 100.0f;	// a
		}
	}
	
	net->SetThreshold(threshold);
	return net;
}



// Create
detectNet* detectNet::Create( NetworkType networkType, float threshold, uint32_t maxBatchSize )
{
	if( networkType == PEDNET_MULTI )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/multiped-500/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/multiped-500/snapshot_iter_178000.caffemodel", "/home/nvidia/catkin_ws/src/people_detect/include/networks/multiped-500/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize );
	else if( networkType == FACENET )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/facenet-120/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/facenet-120/snapshot_iter_24000.caffemodel", NULL, threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize  );
	else if( networkType == PEDNET )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/ped-100/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/ped-100/snapshot_iter_70800.caffemodel", "/home/nvidia/catkin_ws/src/people_detect/include/networks/ped-100/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize  );
	else if( networkType == COCO_AIRPLANE )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Airplane/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Airplane/snapshot_iter_22500.caffemodel", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Airplane/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize );
	else if( networkType == COCO_BOTTLE )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Bottle/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Bottle/snapshot_iter_59700.caffemodel", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Bottle/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize );
	else if( networkType == COCO_CHAIR )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Chair/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Chair/snapshot_iter_89500.caffemodel", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Chair/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize );
	else if( networkType == COCO_DOG )
		return Create("/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Dog/deploy.prototxt", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Dog/snapshot_iter_38600.caffemodel", "/home/nvidia/catkin_ws/src/people_detect/include/networks/DetectNet-COCO-Dog/mean.binaryproto", threshold, DETECTNET_DEFAULT_INPUT, DETECTNET_DEFAULT_COVERAGE, DETECTNET_DEFAULT_BBOX, maxBatchSize );
}


// Create
detectNet* detectNet::Create( int argc, char** argv )
{
	commandLine cmdLine(argc, argv);

	const char* modelName = cmdLine.GetString("model");

	if( !modelName )
	{
		if( argc == 2 )
			modelName = argv[1];
		else if( argc == 4 )
			modelName = argv[3];
		else
			modelName = "pednet";
	}

	//if( argc > 3 )
	//	modelName = argv[3];	

	detectNet::NetworkType type = detectNet::PEDNET_MULTI;

	if( strcasecmp(modelName, "multiped") == 0 || strcasecmp(modelName, "multiped-500") == 0 )
		type = detectNet::PEDNET_MULTI;
	else if( strcasecmp(modelName, "pednet") == 0 || strcasecmp(modelName, "ped-100") == 0 )
		type = detectNet::PEDNET;
	else if( strcasecmp(modelName, "facenet") == 0 || strcasecmp(modelName, "facenet-120") == 0 || strcasecmp(modelName, "face-120") == 0 )
		type = detectNet::FACENET;
	else if( strcasecmp(modelName, "coco-airplane") == 0 || strcasecmp(modelName, "airplane") == 0 )
		type = detectNet::COCO_AIRPLANE;
	else if( strcasecmp(modelName, "coco-bottle") == 0 || strcasecmp(modelName, "bottle") == 0 )
		type = detectNet::COCO_BOTTLE;
	else if( strcasecmp(modelName, "coco-chair") == 0 || strcasecmp(modelName, "chair") == 0 )
		type = detectNet::COCO_CHAIR;
	else if( strcasecmp(modelName, "coco-dog") == 0 || strcasecmp(modelName, "dog") == 0 )
		type = detectNet::COCO_DOG;
	else
	{
		const char* prototxt = cmdLine.GetString("prototxt");
		const char* input    = cmdLine.GetString("input_blob");
		const char* out_cvg  = cmdLine.GetString("output_cvg");
		const char* out_bbox = cmdLine.GetString("output_bbox");
		
		if( !input ) 	input    = DETECTNET_DEFAULT_INPUT;
		if( !out_cvg )  out_cvg  = DETECTNET_DEFAULT_COVERAGE;
		if( !out_bbox ) out_bbox = DETECTNET_DEFAULT_BBOX;
		
		float threshold = cmdLine.GetFloat("threshold");
		
		if( threshold == 0.0f )
			threshold = 0.5f;
		
		int maxBatchSize = cmdLine.GetInt("batch_size");
		
		if( maxBatchSize < 1 )
			maxBatchSize = 2;

		return detectNet::Create(prototxt, modelName, NULL, threshold, input, out_cvg, out_bbox, maxBatchSize);
	}

	// create segnet from pretrained model
	return detectNet::Create(type);
}
	
	
cudaError_t cudaPreImageNetMean( float4* input, size_t inputWidth, size_t inputHeight, float* output, size_t outputWidth, size_t outputHeight, const float3& mean_value );



struct float6 { float x; float y; float z; float w; float v; float u; };
static inline float6 make_float6( float x, float y, float z, float w, float v, float u ) { float6 f; f.x = x; f.y = y; f.z = z; f.w = w; f.v = v; f.u = u; return f; }


inline static bool rectOverlap(const float6& r1, const float6& r2)
{
    return ! ( r2.x > r1.z  
        || r2.z < r1.x
        || r2.y > r1.w
        || r2.w < r1.y
        );
}

static void mergeRect( std::vector<float6>& rects, const float6& rect )
{
	const uint32_t num_rects = rects.size();
	
	bool intersects = false;
	
	for( uint32_t r=0; r < num_rects; r++ )
	{
		if( rectOverlap(rects[r], rect) )
		{
			intersects = true;   

#ifdef DEBUG_CLUSTERING
			printf("found overlap\n");		
#endif

			if( rect.x < rects[r].x ) 	rects[r].x = rect.x;
			if( rect.y < rects[r].y ) 	rects[r].y = rect.y;
			if( rect.z > rects[r].z )	rects[r].z = rect.z;
			if( rect.w > rects[r].w ) 	rects[r].w = rect.w;
			
			break;
		}
			
	} 
	
	if( !intersects )
		rects.push_back(rect);
}


// Detect
bool detectNet::Detect( float* rgba, uint32_t width, uint32_t height, float* boundingBoxes, int* numBoxes, float* confidence )
{
	if( !rgba || width == 0 || height == 0 || !boundingBoxes || !numBoxes || *numBoxes < 1 )
	{
		printf("detectNet::Detect( 0x%p, %u, %u ) -> invalid parameters\n", rgba, width, height);
		return false;
	}

	
	// downsample and convert to band-sequential BGR
	if( CUDA_FAILED(cudaPreImageNetMean((float4*)rgba, width, height, mInputCUDA, mWidth, mHeight,
								  make_float3(104.0069879317889f, 116.66876761696767f, 122.6789143406786f))) )
	{
		printf("detectNet::Classify() -- cudaPreImageNetMean failed\n");
		return false;
	}
	
	// process with GIE
	void* inferenceBuffers[] = { mInputCUDA, mOutputs[OUTPUT_CVG].CUDA, mOutputs[OUTPUT_BBOX].CUDA };
	
	if( !mContext->execute(1, inferenceBuffers) )
	{
		printf(LOG_GIE "detectNet::Classify() -- failed to execute tensorRT context\n");
		*numBoxes = 0;
		return false;
	}
	
	PROFILER_REPORT();

	// cluster detection bboxes
	float* net_cvg   = mOutputs[OUTPUT_CVG].CPU;
	float* net_rects = mOutputs[OUTPUT_BBOX].CPU;
	
	const int ow  = DIMS_W(mOutputs[OUTPUT_BBOX].dims);		// number of columns in bbox grid in X dimension
	const int oh  = DIMS_H(mOutputs[OUTPUT_BBOX].dims);		// number of rows in bbox grid in Y dimension
	const int owh = ow * oh;							// total number of bbox in grid
	const int cls = GetNumClasses();					// number of object classes in coverage map
	
	const float cell_width  = /*width*/ DIMS_W(mInputDims) / ow;
	const float cell_height = /*height*/ DIMS_H(mInputDims) / oh;
	
	const float scale_x = float(width) / float(DIMS_W(mInputDims));
	const float scale_y = float(height) / float(DIMS_H(mInputDims));

#ifdef DEBUG_CLUSTERING	
	printf("input width %i height %i\n", (int)DIMS_W(mInputDims), (int)DIMS_H(mInputDims));
	printf("cells x %i  y %i\n", ow, oh);
	printf("cell width %f  height %f\n", cell_width, cell_height);
	printf("scale x %f  y %f\n", scale_x, scale_y);
#endif
#if 1
	std::vector< std::vector<float6> > rects;
	rects.resize(cls);
	
	// extract and cluster the raw bounding boxes that meet the coverage threshold
	for( uint32_t z=0; z < cls; z++ )
	{
		rects[z].reserve(owh);
		
		for( uint32_t y=0; y < oh; y++ )
		{
			for( uint32_t x=0; x < ow; x++)
			{
				const float coverage = net_cvg[z * owh + y * ow + x];
				
				if( coverage > mCoverageThreshold )
				{
					const float mx = x * cell_width;
					const float my = y * cell_height;
					
					const float x1 = (net_rects[0 * owh + y * ow + x] + mx) * scale_x;	// left
					const float y1 = (net_rects[1 * owh + y * ow + x] + my) * scale_y;	// top
					const float x2 = (net_rects[2 * owh + y * ow + x] + mx) * scale_x;	// right
					const float y2 = (net_rects[3 * owh + y * ow + x] + my) * scale_y;	// bottom 
					
				#ifdef DEBUG_CLUSTERING
					printf("rect x=%u y=%u  cvg=%f  %f %f   %f %f \n", x, y, coverage, x1, x2, y1, y2);
				#endif					
					mergeRect( rects[z], make_float6(x1, y1, x2, y2, coverage, z) );
				}
			}
		}
	}
	
	//printf("done clustering rects\n");
	
	// condense the multiple class lists down to 1 list of detections
	const uint32_t numMax = *numBoxes;
	int n = 0;
	
	for( uint32_t z = 0; z < cls; z++ )
	{
		const uint32_t numBox = rects[z].size();
		
		for( uint32_t b = 0; b < numBox && n < numMax; b++ )
		{
			const float6 r = rects[z][b];
			
			boundingBoxes[n * 4 + 0] = r.x;
			boundingBoxes[n * 4 + 1] = r.y;
			boundingBoxes[n * 4 + 2] = r.z;
			boundingBoxes[n * 4 + 3] = r.w;
			
			if( confidence != NULL )
			{
				confidence[n * 2 + 0] = r.v;	// coverage
				confidence[n * 2 + 1] = r.u;	// class ID
			}
			
			n++;
		}
	}
	
	*numBoxes = n;
#else
	*numBoxes = 0;
#endif
	return true;
}


// DrawBoxes
bool detectNet::DrawBoxes( float* input, float* output, uint32_t width, uint32_t height, const float* boundingBoxes, int numBoxes, int classIndex )
{
	if( !input || !output || width == 0 || height == 0 || !boundingBoxes || numBoxes < 1 || classIndex < 0 || classIndex >= GetNumClasses() )
		return false;
	
	const float4 color = make_float4( mClassColors[0][classIndex*4+0], 
									  mClassColors[0][classIndex*4+1],
									  mClassColors[0][classIndex*4+2],
									  mClassColors[0][classIndex*4+3] );
	
	printf("draw boxes  %i  %i   %f %f %f %f\n", numBoxes, classIndex, color.x, color.y, color.z, color.w);
	
	if( CUDA_FAILED(cudaRectOutlineOverlay((float4*)input, (float4*)output, width, height, (float4*)boundingBoxes, numBoxes, color)) )
		return false;
	
	return true;
}
	

// SetClassColor
void detectNet::SetClassColor( uint32_t classIndex, float r, float g, float b, float a )
{
	if( classIndex >= GetNumClasses() || !mClassColors[0] )
		return;
	
	const uint32_t i = classIndex * 4;
	
	mClassColors[0][i+0] = r;
	mClassColors[0][i+1] = g;
	mClassColors[0][i+2] = b;
	mClassColors[0][i+3] = a;
}
