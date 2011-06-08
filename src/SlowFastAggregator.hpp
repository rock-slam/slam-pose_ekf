#ifndef SLOWFASTAGGREGATOR_HPP
#define SLOWFASTAGGREGATOR_HPP

#include <StreamAligner.hpp>

namespace pose_ekf {
class SlowFastAggregator
{
  
    
  private: 
      /** instance of the aggregator which aligns the readings in time */
	aggregator::StreamAligner *slow_aggr;
	aggregator::StreamAligner *fast_aggr;
      	bool waited_sample_processed;
	bool *waiting_sample; 
	size_t *num_sample_slow; 
	size_t *num_sample_fast; 
	int stream_size; 
	
  public:
    SlowFastAggregator(){}
    void configureSlowFastAggr( aggregator::StreamAligner& slow_aggr,  aggregator::StreamAligner& fast_aggr )
    {
      
	this->slow_aggr = &slow_aggr;
	this->fast_aggr = &fast_aggr; 
	
	stream_size = slow_aggr.getStreamSize();
	
	num_sample_slow = new size_t[stream_size];
	num_sample_fast = new size_t[stream_size];
	
	waiting_sample = new bool[stream_size];
	for ( int i = 0; i < stream_size; i++) 
	    waiting_sample[i] = false;
	
	waited_sample_processed = false; 
	
	
    }
 
    /**
      sample_idx - is the index of the sample processed 
    */ 
    virtual void sampleProcessedSlow( int sample_idx )=0;
    virtual void sampleProcessedFast( int sample_idx )=0;
    
    /**
      process the sample of the slow filter
      return boolean - if there should be a copy or not from the slow filter to the fast filter
      the copy condition is if the slow Filter processed a sample not processed by the fast filter
    */
    bool slowAggrStep( double max_delay )
    {
         for( int i = 0; i < stream_size; i++)
	{
	    
	    const aggregator::StreamStatus &status( slow_aggr->getBufferStatus(i) );
		
	    num_sample_slow[i] = status.buffer_fill; 

	    //if I am waiting for a particular sample 
	    if ( waiting_sample[i] )
	    {
		
		//if I received this sample 
		if( status.buffer_fill != 0)
		{
		    
		    waiting_sample[i] = false;
		    //slow filter will use a sample not used by the fast filter
		    waited_sample_processed = true;
		    
		}
	    }
	    
	}
 
	// then call the streams in the relevant order
	while( slow_aggr->step() )
	{	

	    for ( int i = 0; i < stream_size; i++) 
	    {
		const aggregator::StreamStatus &status( slow_aggr->getBufferStatus(i) );
		
		//this indicate which sample was processed 
		if ( num_sample_slow[i] > status.buffer_fill )
		{
		    num_sample_slow[i] = status.buffer_fill;
		    
		    sampleProcessedSlow(i); 
		    
		    //found the sample that was processed 
		    break; 
		}
		
	    }

	}
	
	if ( slow_aggr->getLatency().toSeconds() >  max_delay )
	{
	    //if there is latency in the slow filter, it means that it is waiting for a sample,
	    //so the number of samples on a buffer for the stream it is waiting is zero. 
	    
	    for( int i = 0; i < stream_size; i++)
	    {
		
		const aggregator::StreamStatus &status( slow_aggr->getBufferStatus(i) );
		
		//if there is 0 samples in the buffer it means I am waiting for this sample 
		if( status.buffer_fill == 0 && !waiting_sample[i] )
		{
		    
		    waiting_sample[i] = true; 
		}
		
	    }
	    
	    if ( waited_sample_processed ) 
	    {
		waited_sample_processed = false; 
		return true; 
	    
	    }
	    else 
	    {
		return false; 
	    }
	}
	else 
	{
	    //even if there is a new sample processed the slow filter aint delayed enouth that the fast filter needs to be processed
	    return false; 
	}
    }
    
    /**
    * process the fast agregator sample if the slow agg has a latency bigger than the max_delay
    */ 
    void fastAggrStep( double max_delay )
    {
      
	if ( slow_aggr->getLatency().toSeconds() >  max_delay )
	{	
	  
	    for( int i = 0; i < stream_size; i++)
	    {
		
		//the number of samples to be processed in the fast filter 
		const aggregator::StreamStatus &status( fast_aggr->getBufferStatus(i) );
		
		num_sample_fast[i] = status.buffer_fill; 
		
	    }
	    
	    while ( fast_aggr->step() )
	    {
		
		for ( int i = 0; i < stream_size; i++) 
		{
		    const aggregator::StreamStatus &status( fast_aggr->getBufferStatus(i) );
		    
		    //this indicate which sample was processed 
		    if ( num_sample_fast[i] > status.buffer_fill )
		    {
			num_sample_fast[i] = status.buffer_fill;
			
			sampleProcessedFast(i); 
			
			//found the sample that was processed 
			break; 
		    }
		
		}
		
	    }
	    

	}
    }
    
};
}

#endif // SLOWFASTAGGREGATOR_H
