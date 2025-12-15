#!/bin/bash
# GPU Performance Benchmark Script for llama-cpp
# Tests token generation speed with gpt-oss-20b model

set -e

MODEL="gpt-oss-20b"
PORT="11435"
API_URL="http://localhost:${PORT}"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}GPU Performance Benchmark${NC}"
echo -e "${BLUE}Model: ${MODEL}${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to get GPU stats
get_gpu_stats() {
    nvidia-smi --query-gpu=power.draw,temperature.gpu,utilization.gpu,clocks.current.graphics,clocks.current.memory,memory.used --format=csv,noheader,nounits
}

# Function to run inference benchmark
run_benchmark() {
    local prompt="$1"
    local n_predict="$2"
    local test_name="$3"
    
    echo -e "${YELLOW}Running: ${test_name}${NC}"
    echo "Prompt length: ${#prompt} chars"
    echo "Tokens to generate: ${n_predict}"
    
    # Get GPU stats before
    echo -e "${GREEN}GPU Before:${NC}"
    get_gpu_stats | awk -F', ' '{printf "  Power: %sW | Temp: %s°C | Util: %s%% | GPU Clock: %sMHz | Mem Clock: %sMHz | VRAM: %sMiB\n", $1, $2, $3, $4, $5, $6}'
    
    # Run inference and capture timing
    START_TIME=$(date +%s.%N)
    
    RESPONSE=$(curl -s "${API_URL}/completion" \
        -H "Content-Type: application/json" \
        -d "{
            \"prompt\": \"${prompt}\",
            \"n_predict\": ${n_predict},
            \"temperature\": 0.7,
            \"top_p\": 0.9,
            \"cache_prompt\": false,
            \"stream\": false
        }")
    
    END_TIME=$(date +%s.%N)
    DURATION=$(echo "$END_TIME - $START_TIME" | bc)
    
    # Get GPU stats after
    echo -e "${GREEN}GPU After:${NC}"
    get_gpu_stats | awk -F', ' '{printf "  Power: %sW | Temp: %s°C | Util: %s%% | GPU Clock: %sMHz | Mem Clock: %sMHz | VRAM: %sMiB\n", $1, $2, $3, $4, $5, $6}'
    
    # Parse response
    TOKENS_PREDICTED=$(echo "$RESPONSE" | jq -r '.tokens_predicted // 0')
    TOKENS_EVALUATED=$(echo "$RESPONSE" | jq -r '.tokens_evaluated // 0')
    TIMINGS=$(echo "$RESPONSE" | jq -r '.timings // {}')
    
    if [ "$TOKENS_PREDICTED" != "0" ] && [ "$TOKENS_PREDICTED" != "null" ]; then
        PROMPT_EVAL_TIME=$(echo "$TIMINGS" | jq -r '.prompt_per_second // 0')
        PREDICT_EVAL_TIME=$(echo "$TIMINGS" | jq -r '.predicted_per_second // 0')
        
        echo -e "${GREEN}Results:${NC}"
        echo "  Total Duration: ${DURATION}s"
        echo "  Tokens Evaluated (prompt): ${TOKENS_EVALUATED}"
        echo "  Tokens Predicted (output): ${TOKENS_PREDICTED}"
        echo -e "  ${BLUE}Prompt Processing: ${PROMPT_EVAL_TIME} tokens/sec${NC}"
        echo -e "  ${BLUE}Token Generation: ${PREDICT_EVAL_TIME} tokens/sec${NC}"
    else
        echo -e "${RED}Error: Failed to get valid response${NC}"
        echo "Response: $RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE"
    fi
    
    echo ""
}

# Check if server is responsive
echo "Checking server health..."
if ! curl -s "${API_URL}/health" > /dev/null; then
    echo -e "${RED}Error: Server at ${API_URL} is not responsive${NC}"
    exit 1
fi
echo -e "${GREEN}Server is healthy${NC}"
echo ""

# Get initial GPU state
echo -e "${YELLOW}Initial GPU State:${NC}"
nvidia-smi --query-gpu=name,persistence_mode,power.limit,power.max_limit,pstate --format=csv,noheader
echo ""

# Test 1: Short generation (fast test)
run_benchmark \
    "What is artificial intelligence? Explain in detail." \
    50 \
    "Test 1: Short Generation (50 tokens)"

sleep 2

# Test 2: Medium generation
run_benchmark \
    "Write a detailed explanation of how neural networks work, including backpropagation and gradient descent." \
    150 \
    "Test 2: Medium Generation (150 tokens)"

sleep 2

# Test 3: Long generation (stress test)
run_benchmark \
    "Write a comprehensive guide on deep learning, covering convolutional neural networks, recurrent neural networks, transformers, and modern architectures like GPT and BERT. Include mathematical foundations and practical applications." \
    300 \
    "Test 3: Long Generation (300 tokens)"

sleep 2

# Test 4: Code generation
run_benchmark \
    "Write a Python class that implements a simple neural network with forward and backward propagation. Include detailed comments." \
    200 \
    "Test 4: Code Generation (200 tokens)"

# Final GPU stats
echo -e "${YELLOW}Final GPU State:${NC}"
nvidia-smi
echo ""

# Check for throttling
echo -e "${YELLOW}Checking Throttling History:${NC}"
nvidia-smi -q -d PERFORMANCE | grep -A 10 "Clocks Event Reasons"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Benchmark Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
