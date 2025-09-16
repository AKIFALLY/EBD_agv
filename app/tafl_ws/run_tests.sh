#!/bin/bash
#
# TAFL Test Runner
# Run all TAFL tests to verify the system is working correctly
#

echo "================================"
echo "Running TAFL Test Suite"
echo "================================"

# Change to the test directory
cd /home/ct/RosAGV/app/tafl_ws/src/tafl/test

# Run the comprehensive verb tests
echo ""
echo "Running TAFL Verb Tests (All 14 tests)..."
echo "----------------------------------------"
python3 test_verbs.py

# Check if the test passed
if [ $? -eq 0 ]; then
    echo ""
    echo "✅ TAFL tests completed successfully!"
    echo "Results saved to: /tmp/tafl_verb_test_results.json"
else
    echo ""
    echo "❌ TAFL tests failed. Please check the output above."
    exit 1
fi

# Optional: Run other tests if they exist and are working
# echo ""
# echo "Running other TAFL tests..."
# python3 test_parser.py
# python3 test_executor.py
# python3 test_validator.py

echo ""
echo "================================"
echo "Test suite execution complete"
echo "================================"