#include "cache.h"
#include "dogfault.h"
#include <assert.h>
#include <ctype.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// DO NOT MODIFY THIS FILE. INVOKE AFTER EACH ACCESS FROM runTrace
void print_result(result r) {
  if (r.status == CACHE_EVICT)
    printf(" [status: miss eviction, victim_block: 0x%llx, insert_block: 0x%llx]",
           r.victim_block_addr, r.insert_block_addr);
  if (r.status == CACHE_HIT)
    printf(" [status: hit]");
  if (r.status == CACHE_MISS)
    printf(" [status: miss, insert_block: 0x%llx]", r.insert_block_addr);
}

/* This is the entry point to operate the cache for a given address in the trace file.
 * First, is increments the global lru_clock in the corresponding cache set for the address.
 * Second, it checks if the address is already in the cache using the "probe_cache" function.
 * If yes, it is a cache hit:
 *     1) call the "hit_cacheline" function to update the counters inside the hit cache 
 *        line, including its lru_clock and access_counter.
 *     2) record a hit status in the return "result" struct and update hit_count 
 * Otherwise, it is a cache miss:
 *     1) call the "insert_cacheline" function, trying to find an empty cache line in the
 *        cache set and insert the address into the empty line. 
 *     2) if the "insert_cacheline" function returns true, record a miss status and the
          inserted block address in the return "result" struct and update miss_count
 *     3) otherwise, if the "insert_cacheline" function returns false:
 *          a) call the "victim_cacheline" function to figure which victim cache line to 
 *             replace based on the cache replacement policy (LRU and LFU).
 *          b) call the "replace_cacheline" function to replace the victim cache line with
 *             the new cache line to insert.
 *          c) record an eviction status, the victim block address, and the inserted block
 *             address in the return "result" struct. Update miss_count and eviction_count.
 */
result operateCache(const unsigned long long address, Cache *cache) {
  result r;
  Set* set = &(cache->sets[cache_set(address, cache)]);
  ++(set->lru_clock);

  if(probe_cache(address, cache) == true){
    hit_cacheline(address, cache);
    cache->hit_count++; 
    r.status = 1;
  }
  else{
    bool insertion = insert_cacheline(address, cache);
    if(insertion == true){
      r.status = 0;
      r.insert_block_addr = address_to_block(address, cache);
      cache->miss_count++;
    }
    else{ 
      unsigned long long victim = victim_cacheline(address, cache);
      replace_cacheline(victim, address, cache);
      r.victim_block_addr = victim; // Already a block address, no need to call address_to_block() function
      r.insert_block_addr = address_to_block(address, cache);
      r.status = 2;
      cache->miss_count++;
      cache->eviction_count++;
    }
  }
  
  return r;
}

// HELPER FUNCTIONS USEFUL FOR IMPLEMENTING THE CACHE
// Given an address, return the block (aligned) address,
// i.e., byte offset bits are cleared to 0
unsigned long long address_to_block(const unsigned long long address,
                                const Cache *cache) {
  unsigned long long block = ((address >> cache->blockBits) << cache->blockBits);
  //Mask out the last 2^b bits for the block aligned address
  return block;
}

// Address looks like: |   TAG   |   INDEX   |  BLOCK OFFSET  |

// Return the cache tag of an address
unsigned long long cache_tag(const unsigned long long address,
                             const Cache *cache) {
  return (address >> (cache->blockBits + cache->setBits));
}

// Return the cache set index of the address
unsigned long long cache_set(const unsigned long long address,
                             const Cache *cache) {
  unsigned long long shifted = address >> cache->blockBits;
  return shifted & ((1ULL << cache->setBits) - 1);    // (1ULL << "value") - 1 = 100000000 - 1 = 0111111111  (useful bitmask)
}

// ++set->lru_clock // Increment the global clock of the set (happens on every cache access (hit or insert))
// currentLine->lru_clock = ++currentSet->lru_clock // Set the accessed line's LRU clock as the new highest value to indicate that it has been accessed

// Check if the address is found in the cache. If so, return true. else return false.

bool probe_cache(const unsigned long long address, const Cache *cache) {
  unsigned long long currentTag = cache_tag(address, cache);
  unsigned long long currentSetIndex = cache_set(address, cache);
  Set* currentSet = &(cache->sets[currentSetIndex]);
  for (int i = 0; i < cache->linesPerSet; i++) {
    Line* currentLine = &(currentSet->lines[i]);
    
    if (currentLine->valid == true && currentLine->tag == currentTag) {
      return true; // Indicate a cache HIT
    }
  }
  return false;
}

// Access address in cache. Called only if probe is successful.
// Update the LRU (least recently used) or LFU (least frequently used) counters.
void hit_cacheline(const unsigned long long address, Cache *cache){
  Set* set = &(cache->sets[cache_set(address, cache)]);
  unsigned long long tag = cache_tag(address, cache);
  for (int i = 0; i < cache->linesPerSet; i++) {
    Line* line = &(set->lines[i]); //Iterate through the set
    if (line->valid == true && line->tag == tag) { //Check for the right line
      if (cache->lfu == 1) { // LFU mode
        line->access_counter++;
      }
      // LRU mode
      line->lru_clock = set->lru_clock; //lru clock increments regardless, as LFU still uses LRU as tiebreaker
      
      return;
    }
  }
 }

/* This function is only called if probe_cache returns false, i.e., the address is
 * not in the cache. In this function, it will try to find an empty (i.e., invalid)
 * cache line for the address to insert. 
 * If it found an empty one:
 *     1) it inserts the address into that cache line (marking it valid).
 *     2) it updates the cache line's lru_clock based on the global lru_clock 
 *        in the cache set and initiates the cache line's access_counter.
 *     3) it returns true.
 * Otherwise, it returns false.  
 */ 
bool insert_cacheline(const unsigned long long address, Cache *cache) {
  Set* set = &(cache->sets[cache_set(address, cache)]);
  unsigned long long tag = cache_tag(address, cache);

  for(int i = 0; i< cache->linesPerSet; i++){ //Iterating through the set
    Line* line = &(set->lines[i]);

    if(line->valid == false){ //If it finds an empty line, set validity to true, change the tag to the inserted tag
                              //The lru clock is updated to set clock, and the access counter is initialized to 1, return true
      line->valid = true;
      line->tag = tag;
      line->block_addr = (address >> cache->blockBits) << cache->blockBits;
      line->lru_clock = set->lru_clock;
      line->access_counter = 1;
      return true;
    }
  }
   return false; //If no empty line is found, return flase
}

//Helper function to reconstruct the block address from a line
unsigned long long line_to_block_address(const Line *line, unsigned long long setIndex, const Cache *cache){
  unsigned long long address = line->tag << (cache->setBits + cache->blockBits); //Move the tag into the right place
  address = address | (setIndex << cache->blockBits); //Add in the set index
  return address;
}

// If there is no empty cacheline, this method figures out which cacheline to replace
// depending on the cache replacement policy (LRU and LFU). It returns the block address
// of the victim cacheline; note we no longer have access to the full address of the victim
unsigned long long victim_cacheline(const unsigned long long address,
                                const Cache *cache) {

  unsigned long long setIndex = cache_set(address, cache);
  Set* set = &(cache->sets[setIndex]);
  Line* least_recent_line = &(set->lines[0]); //Initialize least recent line as the first one
  Line* least_frequent_line = &(set->lines[0]); //Initialize least frequent line as the first line

  for(int i = 1; i < cache->linesPerSet; i++){
    Line* line = &(set->lines[i]);

    if(cache->lfu == 0){ // Least recently used case
      if(line->lru_clock < least_recent_line->lru_clock){
        least_recent_line = line; //If the line we are iterating through has a lower LRU clock(less recent)
                                 //Then our least recent line is set as the iterating line
      }
    }
    else{//Least frequently used case, LFU = 1
      if(line->access_counter < least_frequent_line->access_counter){//If iterating line has a lower access counter than current least frequent line
        least_frequent_line = line;
      }
      else if((line->access_counter == least_frequent_line->access_counter) && line->lru_clock < least_frequent_line->lru_clock){
        //If iterating line has equal access counter, and has lower lru clock (less recent)
        least_frequent_line = line;
      }
    }
  }

  if(cache->lfu == 0){//After iteration, return the LRU or LFU line address, using helper function
    return line_to_block_address(least_recent_line, setIndex, cache);
  }

  else{
    return line_to_block_address(least_frequent_line, setIndex, cache);
  }
}

/* Replace the victim cacheline with the new address to insert. Note for the victim cachline,
 * we only have its block address. For the new address to be inserted, we have its full address.
 * Remember to update the new cache line's lru_clock based on the global lru_clock in the cache
 * set and initiate the cache line's access_counter.
 */
void replace_cacheline(const unsigned long long victim_block_addr, const unsigned long long insert_addr, Cache *cache) {
  unsigned long long currentSetIndex = cache_set(insert_addr, cache);
  unsigned long long newTag = cache_tag(insert_addr, cache);
  Set* currentSet = &(cache->sets[currentSetIndex]);
  for (int i = 0; i < cache->linesPerSet; i++) {
    Line* currentLine = &(currentSet->lines[i]);
    
    if (currentLine->valid == true && currentLine->block_addr == victim_block_addr) { // Have found the victimLine
      currentLine->tag = newTag;
      currentLine->block_addr = (insert_addr >> cache->blockBits) << cache->blockBits;
      currentLine->lru_clock = currentSet->lru_clock;
      currentLine->access_counter = 1;
      return;
    }
  }
}

// allocate the memory space for the cache with the given cache parameters
// and initialize the cache sets and lines.
// Initialize the cache name to the given name 
void cacheSetUp(Cache* cache, char* name) {
  int numSets = 1U << cache->setBits;  // Determine number of sets
  cache->sets = malloc(numSets * sizeof(Set));  // Allocate memory for the "sets" array (that contains a "numSets" quantity of sets)

  for (int i = 0; i < numSets; i++) {
    Set* currentSet = &(cache->sets[i]);
    currentSet->lines = malloc(cache->linesPerSet * sizeof(Line));  // Allocate memory for the "lines" array (that contains a "linesPerSet" quantity of lines)
    currentSet->lru_clock = 0; // Initialize the lru_clock to zero

    for (int j = 0; j < cache->linesPerSet; j++) {  // Initilize each line in the currentSet with default parameters
      Line* currentLine = &(currentSet->lines[j]);
      currentLine->block_addr = 0;
      currentLine->valid = 0;
      currentLine->tag = 0;
      currentLine->lru_clock = 0;
      currentLine->access_counter = 0;
    }

  }
  cache->name = name;
}

// deallocate the memory space for the cache
void deallocate(Cache *cache) {
  int numSets = 1U << cache->setBits;  // Determine number of sets

  for (int i = 0; i < numSets; i++) {
    Set* currentSet = &(cache->sets[i]);
    free(currentSet->lines);  // Deallocate lines array
    currentSet->lines = NULL;  // Set to NULL to avoid dangling pointers
  }
  free(cache->sets);  // Deallocate sets array
  cache->sets = NULL;  // Set to NULL to avoid dangling pointers
}

// print out summary stats for the cache
void printSummary(const Cache *cache) {
  printf("%s hits: %d, misses: %d, evictions: %d\n", cache->name, cache->hit_count,
         cache->miss_count, cache->eviction_count);
}