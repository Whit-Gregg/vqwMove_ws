#pragma once

namespace vqw {
    
    class ValueDistribution
    {
      public:

        ValueDistribution()
        {
            // Initialize the value distribution with zeros
            for (int i = 0; i < 101; ++i) {
                value[i] = 0;
            }
        }

        static const int MAX_VALUE = 100;

        void addValue(int val)
        {
            // Ensure the value is within the range 0-100
            if (val < 0) val = 0;
            if (val > 100) val = 100;

            // Increment the count for the given value
            value[val]++;
        }

        long getValue(int val) const
        {
            // Ensure the value is within the range 0-100
            if (val < 0) val = 0;
            if (val > 100) val = 100;

            // Return the count for the given value
            return value[val];
        }

      private:
        long value[101];
    };

}  // namespace vqw

