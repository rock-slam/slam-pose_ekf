#ifndef SLOWFASTAGGREGATOR_HPP
#define SLOWFASTAGGREGATOR_HPP

#include <aggregator/StreamAligner.hpp>

namespace pose_ekf {
class SlowFastAggregator
{
  
    
  private: 
      /** instance of the aggregator which aligns the readings in time */
	aggregator::StreamAligner *slow_aggr;
	aggregator::StreamAligner *fast_aggr;
      	
	/** The number of streams in the aggr*/
	int stream_size; 

	/** The previous number of streams processed in the slow aggregator */ 
	size_t *prev_num_processed_streams_slow; 
	/** if there needs to be a copy */ 
	bool copy; 
	
  public:
      
    SlowFastAggregator(){}
    
    void configureSlowFastAggr( aggregator::StreamAligner& slow_aggr,  aggregator::StreamAligner& fast_aggr )
    {
      
	this->slow_aggr = &slow_aggr;
	this->fast_aggr = &fast_aggr; 
	
	stream_size = slow_aggr.getStreamSize();
	
	prev_num_processed_streams_slow = new size_t[stream_size];
	for(int i = 0; i < stream_size; i++) 
	{
	    prev_num_processed_streams_slow[i] = 0; 
	}
	
	copy = false; 
	
    }
    
    /** 
     * If there is a copy between the slow and fast filter this function is called. 
     */ 
   virtual void copyState()=0; 
    
    /**
     * Proces the slow and fast aggregator and copy between them if needed. 
     *The aggregators are implicitly copied  
     * The copy is triggered when the slow filter process a stream that was dropped by the fast filter. 
     */
    void step()
    {
	
	// call the slow aggregator streams in the relevant order
	while( slow_aggr->step() );
	
	//this logic is for determining if a copy is needed, so only triggered if there is no pending copy 
	if(!copy)
	{
	    //Check which streams were dropped by the fast filter, but weren't dropped by the slow filter.
	    for( int i = 0; i < stream_size; i++)
	    {
		const aggregator::StreamStatus &status_fast( fast_aggr->getBufferStatus(i) );
		const aggregator::StreamStatus &status_slow( slow_aggr->getBufferStatus(i) );
		
		int total_stream_dropped_fast =  status_fast.samples_dropped_buffer_full + status_fast.samples_dropped_late_arriving;
		int total_stream_dropped_slow =  status_slow.samples_dropped_buffer_full + status_slow.samples_dropped_late_arriving;
		int current_dif = total_stream_dropped_fast - total_stream_dropped_slow; 
		//verify if a stream was dropped by the fast filter, but wasen't dropped by the slow filter
		if( current_dif > 0 )
		{
		    //verify if that stream of that type was procces by the slow filter 
		    if( prev_num_processed_streams_slow[i] < status_slow.samples_processed)
		    {
			//if the stream process was of a type dropped by the fast filter there need to be a copy
			copy = true; 
			break; 
		    }
		}

		prev_num_processed_streams_slow[i] = status_slow.samples_processed;

	    }
	    
	}
	 
/*	std::cout << " MUST COPY " << copy << std::endl; */
	//verify if there is a need to procces the fast aggregator 
	if ( slow_aggr->getLatency().toSeconds() >  fast_aggr->getTimeOut().toSeconds() )
	{
	    if( copy ) 
	    {
		copy = false; 
		fast_aggr->copyState( *slow_aggr );
		copyState(); 
		
		//since there was a copy this get the new ammount of stream dropped 
		for( int i = 0; i < stream_size; i++)
		{
			const aggregator::StreamStatus &status_slow( slow_aggr->getBufferStatus(i) );
			prev_num_processed_streams_slow[i] = status_slow.samples_processed;
		}
	    }
	while( fast_aggr->step() );

	}

    }

    
};
}

#endif // SLOWFASTAGGREGATOR_H
