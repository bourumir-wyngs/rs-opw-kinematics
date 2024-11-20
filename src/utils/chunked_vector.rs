use rayon::prelude::*;

/// Structure supporting fast expansion without much data copying.
pub struct ChunkedVec<T> {
    chunks: Vec<Vec<T>>, // Internal chunked storage
    chunk_size: usize,
}

impl<T> ChunkedVec<T> {
    pub fn new(chunk_size: usize) -> Self {
        Self {
            chunks: Vec::new(),
            chunk_size,
        }
    }

    pub fn push(&mut self, value: T) {
        if self.chunks.is_empty() || self.chunks.last().unwrap().len() == self.chunk_size {
            self.chunks.push(Vec::with_capacity(self.chunk_size));
        }
        self.chunks.last_mut().unwrap().push(value);
    }

    // Sequential iterator over all elements
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.chunks.iter().flat_map(|chunk| chunk.iter())
    }

    // Parallel iterator over all elements
    pub fn par_iter(&self) -> impl ParallelIterator<Item = &T>
    where
        T: Sync,
    {
        self.chunks.par_iter().flat_map(|chunk| chunk.par_iter())
    }

    // Parallel iterator over chunks
    pub fn par_chunks(&self) -> impl ParallelIterator<Item = &Vec<T>>
    where
        T: Sync,
    {
        self.chunks.par_iter()
    }

    /// Returns the total number of elements in the `ChunkedVec`.
    pub fn len(&self) -> usize {
        self.chunks.iter().map(|chunk| chunk.len()).sum()
    }

    /// Accesses an element by index, similar to `Vec`.
    pub fn get(&self, index: usize) -> &T {
        let mut remaining_index = index;
        for chunk in &self.chunks {
            if remaining_index < chunk.len() {
                return &chunk[remaining_index];
            }
            remaining_index -= chunk.len();
        }
        panic!("Chunk out of bounds");
    }

    /// Mutable access to an element by index.
    pub fn get_mut(&mut self, index: usize) -> &mut T {
        let mut remaining_index = index;
        for chunk in &mut self.chunks {
            if remaining_index < chunk.len() {
                return &mut chunk[remaining_index];
            }
            remaining_index -= chunk.len();
        }
        panic!("Chunk out of bounds");
    }

    /// Indexing operator for immutable access.
    /// Panics if the index is out of bounds.
    pub fn index(&self, index: usize) -> &T {
        self.get(index)
    }

    /// Indexing operator for mutable access.
    /// Panics if the index is out of bounds.
    pub fn index_mut(&mut self, index: usize) -> &mut T {
        self.get_mut(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_and_iter() {
        let mut chunked_vec = ChunkedVec::new(3);
        chunked_vec.push(1);
        chunked_vec.push(2);
        chunked_vec.push(3);
        chunked_vec.push(4);

        let collected: Vec<_> = chunked_vec.iter().cloned().collect();
        assert_eq!(collected, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_chunking_behavior() {
        let mut chunked_vec = ChunkedVec::new(2);
        chunked_vec.push(1);
        chunked_vec.push(2);
        chunked_vec.push(3);

        assert_eq!(chunked_vec.chunks.len(), 2); // Two chunks: [1, 2], [3]
        assert_eq!(chunked_vec.chunks[0], vec![1, 2]);
        assert_eq!(chunked_vec.chunks[1], vec![3]);
    }

    #[test]
    fn test_parallel_sum() {
        let mut chunked_vec = ChunkedVec::new(4);
        for i in 1..=10 {
            chunked_vec.push(i);
        }

        // Parallel sum using par_iter
        let parallel_sum: i32 = chunked_vec.par_iter().sum();

        // Sequential sum for verification
        let sequential_sum: i32 = chunked_vec.iter().sum();

        assert_eq!(parallel_sum, sequential_sum);
        assert_eq!(parallel_sum, 55); // 1 + 2 + ... + 10
    }

    #[test]
    fn test_parallel_chunk_processing() {
        let mut chunked_vec = ChunkedVec::new(3);
        for i in 1..=9 {
            chunked_vec.push(i);
        }

        // Process chunks in parallel
        let chunk_sums: Vec<i32> = chunked_vec
            .par_chunks()
            .map(|chunk| chunk.iter().sum::<i32>())
            .collect();

        assert_eq!(chunk_sums, vec![6, 15, 24]); // Chunks: [1, 2, 3], [4, 5, 6], [7, 8, 9]
    }

    #[test]
    fn test_empty_chunked_vec() {
        let chunked_vec: ChunkedVec<i32> = ChunkedVec::new(4);

        // Ensure there are no chunks
        assert_eq!(chunked_vec.chunks.len(), 0);

        // Iterators should produce no elements
        assert_eq!(chunked_vec.iter().count(), 0);
        assert_eq!(chunked_vec.par_iter().count(), 0);
        assert_eq!(chunked_vec.par_chunks().count(), 0);
    }

    #[test]
    fn test_single_chunk() {
        let mut chunked_vec = ChunkedVec::new(10);
        for i in 1..=5 {
            chunked_vec.push(i);
        }

        assert_eq!(chunked_vec.chunks.len(), 1); // Only one chunk since 5 < 10
        assert_eq!(chunked_vec.chunks[0], vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_large_chunked_vec() {
        let mut chunked_vec = ChunkedVec::new(100);
        for i in 0..1000 {
            chunked_vec.push(i);
        }

        // Ensure correct number of chunks
        assert_eq!(chunked_vec.chunks.len(), 10);

        // Check total number of elements via parallel iterator
        let total_elements: usize = chunked_vec.par_iter().count();
        assert_eq!(total_elements, 1000);

        // Verify the first and last chunks
        assert_eq!(chunked_vec.chunks.first().unwrap().len(), 100);
        assert_eq!(chunked_vec.chunks.last().unwrap().len(), 100);
    }

    #[test]
    fn test_push_and_size() {
        let mut chunked_vec = ChunkedVec::new(3);
        for i in 0..10 {
            chunked_vec.push(i);
        }
        assert_eq!(chunked_vec.len(), 10);
    }

    #[test]
    fn test_get() {
        let mut chunked_vec = ChunkedVec::new(3);
        for i in 0..10 {
            chunked_vec.push(i);
        }
        assert_eq!(chunked_vec.get(0), &0);
        assert_eq!(chunked_vec.get(5), &5);
        assert_eq!(chunked_vec.get(9), &9);
    }

    #[test]
    fn test_get_mut() {
        let mut chunked_vec = ChunkedVec::new(3);
        for i in 0..10 {
            chunked_vec.push(i);
        }
        let value = chunked_vec.get_mut(5);
        *value *= 2;
        assert_eq!(chunked_vec.get(5), &10);
    }

    #[test]
    fn test_index() {
        let mut chunked_vec = ChunkedVec::new(3);
        for i in 0..10 {
            chunked_vec.push(i);
        }
        assert_eq!(*chunked_vec.index(0), 0);
        assert_eq!(*chunked_vec.index(5), 5);
        assert_eq!(*chunked_vec.index(9), 9);
    }

    #[test]
    fn test_index_mut() {
        let mut chunked_vec = ChunkedVec::new(3);
        for i in 0..10 {
            chunked_vec.push(i);
        }
        *chunked_vec.index_mut(5) *= 2;
        assert_eq!(*chunked_vec.index(5), 10);
    }

    #[test]
    fn test_large_data() {
        let mut chunked_vec = ChunkedVec::new(100);
        for i in 0..1000 {
            chunked_vec.push(i);
        }

        // Check total size
        assert_eq!(chunked_vec.len(), 1000);

        // Verify first and last elements
        assert_eq!(*chunked_vec.index(0), 0);
        assert_eq!(*chunked_vec.index(999), 999);
    }

    #[test]
    fn test_empty_chunked_vec_2() {
        let chunked_vec: ChunkedVec<i32> = ChunkedVec::new(3);
        assert_eq!(chunked_vec.len(), 0);
    }
}
