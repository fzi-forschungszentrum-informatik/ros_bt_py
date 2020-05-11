#include <something.h>

Something::Something()
{
}

bool Something::this_should_be_tested(int in)
{
    if (in > 0)
    {
        return true;
    }
    return false;
}