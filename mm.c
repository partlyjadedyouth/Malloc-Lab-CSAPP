/*
 
 Implemented segragated free list
 
 */

/*
 
 Block Information
 Reference: https://github.com/mightydeveloper/Malloc-Lab/blob/master/mm.c
 
    A : Allocation tag (1: true, 0:false)
     
     < Allocated Block >
     
     
                 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
     Header :   |                              size of the block                                             | A|
        bp ---> +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
                |                                                                                               |
                |                                                                                               |
                .                              Payload and padding                                              .
                .                                                                                               .
                .                                                                                               .
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
     Footer :   |                              size of the block                                             | A|
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
     
     
     < Free block >
     
                 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
     Header :   |                              size of the block                                             | A|
        bp ---> +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
                |                        pointer to its predecessor in Segregated list                          |
    bp+WSIZE--> +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
                |                        pointer to its successor in Segregated list                            |
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
                .                                                                                               .
                .                                                                                               .
                .                                                                                               .
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
     Footer :   |                              size of the block                                             | A|
                +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 
 
    Free blocks will be saved in array 'void *seg_list[LEN]'.
    
*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

/* Default Macros */
/* single word (4) or double word (8) alignment */
#define ALIGNMENT 8
/* rounds up to the nearest multiple of ALIGNMENT */
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~0x7)
#define SIZE_T_SIZE (ALIGN(sizeof(size_t)))

/*
 * Macros and global variables to manipulate seglist
 * Most of the macros written under this field came from the textbook.
 * (Figure 9.43)
 */

#define WSIZE 4          // Word and header - footer size
#define DSIZE 8          // Double word size
#define CHUNKSIZE (1<<6)   // Extend heap by this amount
#define LEN 20  // Length of seg_list

#define MAX(x, y) ((x) > (y) ? (x) : (y))   // Returns a value that is bigger than the other

#define PACK(size, alloc) ((size) | (alloc))    // Pack a size and allocation tag into a word
#define GET(p) (*(unsigned int *)(p))    // Read a word at address p
#define PUT(p, val) (*(unsigned int *)(p) = (val)) // Write a word at address p

#define GET_SIZE(p)  (GET(p) & ~0x7) // Read the size from address p
#define GET_ALLOC(p) (GET(p) & 0x1) // Read the allocation tag from address p

#define HDRP(ptr) ((char *)(ptr) - WSIZE) // Compute address of its header
#define FTRP(ptr) ((char *)(ptr) + GET_SIZE(HDRP(ptr)) - DSIZE) // Compute address of its footer

#define RIGHT_BLKP(ptr) ((char *)(ptr) + GET_SIZE((char *)(ptr) - WSIZE)) // Compute address of the block that is placed physically right(i.e. next) side of the block
#define LEFT_BLKP(ptr) ((char *)(ptr) - GET_SIZE((char *)(ptr) - DSIZE)) // Compute address of the block that is placed physically left(i.e. previous) side of the block

#define PRED_PTR(ptr) ((char *)(ptr)) // Compute address of predecessor block
#define SUCC_PTR(ptr) ((char *)(ptr) + WSIZE) // Compute address of successor block

#define PRED_BLK(ptr) (*(char **)(ptr)) // Compute address of predecessor on the seglist
#define SUCC_BLK(ptr) (*(char **)(SUCC_PTR(ptr))) // Compute address of successor on the seglist
#define SET_PTR(p, ptr) (*(unsigned int *)(p) = (unsigned int)(ptr)) // Store predecessor or successor pointer for free blocks

void *seg_list[LEN]; // Segragated Free List
char *heap_listp; // Pointer to beginning of heap

/*
 * End of macros and global variables
 */



/*
 * Functions
 * Most of the functions written under this field came from the textbook.
 */

/*
 * Prototypes
 */
static void *extend_heap(size_t size); // Extends heap with a new free block
static void *coalesce(void *bp);   // Coalesces bp if needed
static void insert_node(void *ptr, size_t size);  // Inserts bp into the seglist
static void delete_node(void *ptr); // Deletes bp from the seglist
static void place(void *ptr, size_t asize); // Places block of size asize at the beginning of the free block bp.
//static int mm_check(void);    // For debugging


static void *extend_heap(size_t words)
{
    void *bp;   // Block pointer
    size_t asize = ALIGN(words);    // Adjusted block size
    
    // Return NULL if mem_sbrk returns -1
    if ((bp = mem_sbrk(asize)) == (void *)-1) {
        return NULL;
    }
    
    /*
     Set the header and footer
     Insert epilogue block to the end
     Push bp into the seglist
     */
    PUT(HDRP(bp), PACK(asize, 0));
    PUT(FTRP(bp), PACK(asize, 0));
    PUT(HDRP(RIGHT_BLKP(bp)), PACK(0, 1)); // Epilogue block
    insert_node(bp, asize); // Insert bp into the seglist

    return coalesce(bp); // Coalesce seglist if needed
}


static void *coalesce(void *bp)
{
    size_t is_left_alloc = GET_ALLOC(HDRP(LEFT_BLKP(bp)));
    size_t is_right_alloc = GET_ALLOC(HDRP(RIGHT_BLKP(bp)));
    size_t size = GET_SIZE(HDRP(bp));
    
    // Case 1: Both left and right blocks are allocated
    // -> Do not coalesce
    if (is_left_alloc && is_right_alloc) {
        return bp;
    }
    // Case 2: Left block is allocated, while right block is a free block
    // -> Coalesce with the right block
    else if (is_left_alloc && !is_right_alloc) {
        delete_node(bp);
        delete_node(RIGHT_BLKP(bp));
        
        size += GET_SIZE(HDRP(RIGHT_BLKP(bp)));
        PUT(HDRP(bp), PACK(size, 0));
        PUT(FTRP(bp), PACK(size, 0));
    }
    // Case 3: Right block is allocated, while left block is a free block
    // -> Coalesce with the left block
    else if (!is_left_alloc && is_right_alloc) {
        delete_node(bp);
        delete_node(LEFT_BLKP(bp));
        
        size += GET_SIZE(HDRP(LEFT_BLKP(bp)));
        PUT(FTRP(bp), PACK(size, 0));
        PUT(HDRP(LEFT_BLKP(bp)), PACK(size, 0));
        bp = LEFT_BLKP(bp);
    }
    // Case 4: Both left and right blocks are free
    // -> Coalesce all blocks
    else {
        delete_node(bp);
        delete_node(LEFT_BLKP(bp));
        delete_node(RIGHT_BLKP(bp));
        
        size += GET_SIZE(HDRP(LEFT_BLKP(bp))) + GET_SIZE(HDRP(RIGHT_BLKP(bp)));
        PUT(HDRP(LEFT_BLKP(bp)), PACK(size, 0));
        PUT(FTRP(RIGHT_BLKP(bp)), PACK(size, 0));
        bp = LEFT_BLKP(bp);
    }
    
    insert_node(bp, size);  // Insert bp into the seglist
    return bp;
}


static void insert_node(void *bp, size_t size) {
    /*
     * Seglist is sliced by the power of 2.
     * That is, after removing the lsb of 'size' by logically shifting right,
     * we get idx of seglist where bp should be inserted.
     */
    int idx = 0;    // Index of the seglist
    while ((idx < LEN - 1) && (size > 1)) {
        size >>= 1;
        idx++;
    }
    
    void *prev_ptr = NULL;  // bp's predecessor
    void *next_ptr = seg_list[idx]; // bp's successor
    // Traverse seg_list[idx] to find the place to be inserted
    while ((next_ptr != NULL) && (size > GET_SIZE(HDRP(next_ptr)))) {
        prev_ptr = next_ptr;
        next_ptr = SUCC_BLK(next_ptr);
    }
    
    // Set pred and succ (size-ascending order)
    // Case 1: The list is not empty
    // -> Needs manipulation on the predecessor and successor
    if (next_ptr != NULL) {
        // Case 1-1: bp has pred and succ
        if (prev_ptr != NULL) {
            SET_PTR(SUCC_PTR(bp), next_ptr);
            SET_PTR(PRED_PTR(bp), prev_ptr);
            SET_PTR(SUCC_PTR(prev_ptr), bp);
            SET_PTR(PRED_PTR(next_ptr), bp);
        }
        // Case 1-2: bp has to be inserted at the head of seg_list[idx]
        else {
            SET_PTR(SUCC_PTR(bp), next_ptr);
            SET_PTR(PRED_PTR(bp), NULL);
            SET_PTR(PRED_PTR(next_ptr), bp);
            seg_list[idx] = bp;
        }
    }
    else {
        // Case 1-3: bp has to be inserted at the tail of seg_list[idx]
        if (prev_ptr != NULL) {
            SET_PTR(SUCC_PTR(bp), NULL);
            SET_PTR(SUCC_PTR(prev_ptr), bp);
            SET_PTR(PRED_PTR(bp), prev_ptr);
        }
        // Case 2: The list is empty
        else {
            SET_PTR(SUCC_PTR(bp), NULL);
            SET_PTR(PRED_PTR(bp), NULL);
            seg_list[idx] = bp;
        }
    }
    
    return;
}


static void delete_node(void *bp) {
    int idx = 0;
    size_t size = GET_SIZE(HDRP(bp));
    
    // Find idx of seg_list
    while ((idx < LEN - 1) && (size > 1)) {
        size >>= 1;
        idx++;
    }
    
    
    if (SUCC_BLK(bp) != NULL) {
        // Case 1: bp has pred and succ
        if (PRED_BLK(bp) != NULL) {
            SET_PTR(SUCC_PTR(PRED_BLK(bp)), SUCC_BLK(bp));
            SET_PTR(PRED_PTR(SUCC_BLK(bp)), PRED_BLK(bp));
        }
        // Case 2: bp has succ and no pred i.e. bp is the head
        else {
            SET_PTR(PRED_PTR(SUCC_BLK(bp)), NULL);
            seg_list[idx] = SUCC_BLK(bp);
        }
    }
    else {
        // Case 3: bp has pred and no succ i.e. bp is the tail
        if (PRED_BLK(bp) != NULL) {
            SET_PTR(SUCC_PTR(PRED_BLK(bp)), NULL);
        }
        // Case 4: bp has no pred and succ i.e. bp is the only element in seg_list[idx]
        else {
            seg_list[idx] = NULL;
        }
    }
    
    return;
}


static void place(void *bp, size_t asize)
{
    size_t csize = GET_SIZE(HDRP(bp));
    size_t remainder = csize - asize;
    
    delete_node(bp);

    if (remainder >= DSIZE * 2) {   // DSIZE * 2 : Minimum block size
        // Split block
        PUT(HDRP(bp), PACK(asize, 1));
        PUT(FTRP(bp), PACK(asize, 1));
        bp = RIGHT_BLKP(bp);    // Change bp address
        PUT(HDRP(bp), PACK(remainder, 0));
        PUT(FTRP(bp), PACK(remainder, 0));
        insert_node(bp, remainder);
    }
    else {
        // Do not split block
        PUT(HDRP(bp), PACK(csize, 1));
        PUT(FTRP(bp), PACK(csize, 1));
    }
}


/*
 * mm_init - initialize the malloc package.
 */
int mm_init(void)
{
    int idx;
    
    
    // Initialize segregated free lists
    for (idx = 0; idx < LEN; idx++) {
        seg_list[idx] = NULL;
    }
    
    // Allocate memory for the initial empty heap
    if ((long)(heap_listp = mem_sbrk(2 * DSIZE)) == -1)
        return -1;
    
    PUT(heap_listp, 0);                            // Alignment padding
    PUT(heap_listp + (1 * WSIZE), PACK(DSIZE, 1)); // Prologue header
    PUT(heap_listp + (2 * WSIZE), PACK(DSIZE, 1)); // Prologue footer
    PUT(heap_listp + (3 * WSIZE), PACK(0, 1));     // Epilogue header
    heap_listp += (DSIZE);  // Prologue
    
    if (extend_heap(CHUNKSIZE) == NULL) {
        return -1;
    }
    
    return 0;
}


/*
 * mm_malloc - Allocate a block by incrementing the brk pointer.
 *             Always allocate a block whose size is a multiple of the alignment.
 */
void *mm_malloc(size_t size)
{
    size_t asize;
    size_t extendsize; // Heap should be extended to this amount when there's no space
    void *bp = NULL;
    
    // Deal with spurious requests
    if (heap_listp == 0) {
        mm_init();
    }
    if (size == 0) {
        return NULL;
    }
    
    // Initialize asize i.e. adjust block size
    if (size <= DSIZE) {
        asize = 2 * DSIZE;
    }
    else {
        asize = ALIGN(size+DSIZE);
    }
    
    // Search for the freed block
    // Find idx and traverse seg_list[idx]
    int idx = 0;
    size_t searchsize = asize;
    while (idx < LEN) {
        if ((idx == LEN - 1) || ((searchsize <= 1) && (seg_list[idx] != NULL))) {
            bp = seg_list[idx];
            // Ignore blocks that are too small or marked with the reallocation bit
            while ((bp != NULL) && (asize > GET_SIZE(HDRP(bp)))) {
                bp = SUCC_BLK(bp);
            }
            if (bp != NULL) {
                break;
            }
        }
        searchsize >>= 1;
        idx++;
    }
    
    // if free block is not found, extend the heap
    if (bp == NULL) {
        extendsize = MAX(asize, CHUNKSIZE);
        
        if ((bp = extend_heap(extendsize)) == NULL) {
            return NULL;
        }
    }
    
    // Place block
    place(bp, asize);
    
    return bp;
}


/*
 * mm_free - Freeing a block
 */
void mm_free(void *bp)
{
    size_t size = GET_SIZE(HDRP(bp));
 
    PUT(HDRP(bp), PACK(size, 0));   // Header
    PUT(FTRP(bp), PACK(size, 0));   // Footer
    
    insert_node(bp, size);  // Insert ptr into the seglist
    coalesce(bp);
    
    return;
}


/*
 * mm_realloc - Implemented simply in terms of mm_malloc and mm_free
 *
 * Role : The mm_realloc routine returns a pointer to an allocated
 *        region of at least size bytes with constraints.
 */
void *mm_realloc(void *bp, size_t size)
{
    // If bp is NULL, realloc is just malloc.
    if (bp == NULL) {
        return mm_malloc(size);
    }
    
    // If size == 0, just free the block and return NULL.
    if (size == 0) {
        mm_free(bp);
        return NULL;
    }
    
    // If a block has the appropriate size, use the same block.
    if (size < GET_SIZE(HDRP(bp)) - DSIZE) {
        return bp;
    }
    
    size_t oldsize;
    void *newptr;
    void *right = RIGHT_BLKP(bp);
    size_t size_of_right = GET_SIZE(HDRP(right));
    size_t is_right_alloc = GET_ALLOC(HDRP(right));
    
    // Coalesce bp and right if possible
    size_t asize = (GET_SIZE(HDRP(right)) + GET_SIZE(HDRP(bp)));
    if (!is_right_alloc && size <= asize - DSIZE) {
        delete_node(right);
        PUT(HDRP(bp), PACK(asize, 1));
        PUT(FTRP(bp), PACK(asize, 1));
        return bp;
    }
    
    // If right is free and size_of_right is not 0
    if (!is_right_alloc && !size_of_right) {
        size_t remainder = GET_SIZE(HDRP(bp)) + size_of_right - size;
        size_t extendsize;
        
        if (remainder < 0) {
            extendsize = MAX(-remainder, CHUNKSIZE);
            if (extend_heap(extendsize) == NULL) {
                return NULL;
            }
            remainder += extendsize;
        }
        
        delete_node(right);
        PUT(HDRP(bp), PACK(GET_SIZE(HDRP(bp)) + remainder, 1));
        PUT(FTRP(bp), PACK(GET_SIZE(HDRP(bp)) + remainder, 1));
        return bp;
    }
    
    newptr = mm_malloc(size);
    // If realloc failed. return NULL
    if (!newptr) {
        return NULL;
    }
    
    // Copy the old data
    oldsize = GET_SIZE(HDRP(bp));
    if (size < oldsize) {
        oldsize = size;
    }
    memcpy(newptr, bp, oldsize);
    mm_free(bp);
    
    return newptr;
}

/*
 * mm_check : check the heap for consistency
 *
 * Things that should be checked...
 * 1. Is every block in the free list marked as free?
 * 2. Are there any contiguous free blocks that somehow escaped coalescing?
 * 3. Is every free block actually in the free list?
 * 4. Do the pointers in the free list point to valid free blocks?
 * 5. Do any allocated blocks overlap?
 * 6. Do the pointers in a heap block point to valid heap addresses?
 *
 * Returns 0 if the heap is inconsistent, else return 1
 */
// static int mm_check(void) {
//    void *start = mem_heap_lo();
//    void *end = mem_heap_hi();
//
//    // Counters for number of free blocks in seg_list and heap
//    int seg_count = 0;
//    int heap_count = 0;
//    int i;
//
//    // Traverse seg_list
//    for (i = 0; i < LEN; i++) {
//        void *bp = seg_list[i];
//        while (bp) {
//            // 1. Is every block in the free list marked as free?
//            if (GET_ALLOC(HDRP(bp)) == 1 || GET_ALLOC(FTRP(bp)) == 1) {
//                printf("FREE BLOCKS ARE TAGGED AS ALLOCATED\n");
//                return 0;
//            }
//            // 2. Are there any contiguous free blocks that somehow escaped coalescing?
//            if ((LEFT_BLKP(bp) > start) && (GET_ALLOC(HDRP(LEFT_BLKP(bp))) == 0)) {
//                printf("CONTIGUOUS FREE BLOCKS THAT ESCAPED COALESCING DETECTED\n");
//                return 0;
//            }
//            if ((RIGHT_BLKP(bp) < end) && (GET_ALLOC(HDRP(RIGHT_BLKP(bp))) == 0)) {
//                printf("CONTIGUOUS FREE BLOCKS THAT EXCAPED COALESCING DETECTED\n");
//                return 0;
//            }
//            // 4. Do the pointers in the free list point to valid free blocks?
//            if ((heap_listp > bp) | (bp > end)) {
//                printf("FREE BLOCKS ARE NOT IN THE HEAP\n");
//                return 0;
//            }
//            seg_count++;
//            bp = SUCC_PTR(bp);
//        }
//    }
//
//    // Traverse heap
//    void *ptr = heap_listp;
//    while (ptr != NULL && GET_SIZE(HDRP(ptr)) != 0) {
//        if (GET_ALLOC(HDRP(ptr))) {
//            void *right = RIGHT_BLKP(ptr);
//            // 5. Do any allocated blocks overlap?
//            if (ptr + GET_SIZE(HDRP(ptr)) - WSIZE >= right) {
//                printf("ALLOCATED BLOCKS ARE OVERLAPPING\n");
//                return 0;
//            }
//        }
//        else {
//            heap_count++;
//        }
//        ptr = RIGHT_BLKP(ptr);
//    }
//
//    // 3. Is every free block actually in the free list?
//    if (seg_count != heap_count) {
//        printf("%d FREE BLOCKS ARE NOT IN THE SEG_LIST\n", heap_count - seg_count);
//        return 0;
//    }
//
//    // 6. Do the pointers in a heap block point to valid heap addresses?
//    if (end >= mem_heapsize() + start) {
//        printf("MEM_HEAP_HI DOES NOT POINTING TO THE LAST BYTE OF THE HEAP\n");
//        return 0;
//    }
//
//    ptr = start;
//    while (GET_SIZE(HDRP(ptr)) > 0) {
//        if (ptr > end || ptr < start) {
//            printf("POINTER DOES NOT POINTING TO VALID HEAP ADDRESS\n");
//            return 0;
//        }
//        if (GET(HDRP(ptr)) != GET(FTRP(ptr))) {
//            printf("HEADER AND FOOTER IS DIFFERENT\n");
//            return 0;
//        }
//        ptr = RIGHT_BLKP(ptr);
//    }
//
//    return 1;
//}
