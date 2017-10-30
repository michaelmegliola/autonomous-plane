#ifndef __STEAM_ENGINE_LPF_H__
#define __STEAM_ENGINE_LPF_H__

/*  =============================================================================
 *  Low Pass Filter
 *  
 *  Reduces noise in sensor readings, at the expense of muting
 *  or delaying the effect of rapid transient changes.
 */
class SteamEngineLPF {

private:
	float *buf;
	int i;
	int size;

public:
	SteamEngineLPF(int buffersize);
	void update(float val);
	float getFilteredVal();
};

#endif
