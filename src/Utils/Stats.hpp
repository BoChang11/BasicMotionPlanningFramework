#ifndef MP_STATS_HPP_
#define MP_STATS_HPP_

#include "Utils/Map.hpp"
#include <cstdio>
#include <string>

namespace MP
{
  static inline void PrintVector(std::vector<int> v)
  {
    for (int i = 0 ; i < v.size(); i++)
    {
      printf("%d ",v[i] );
    }
    printf("\n" );
  }

  static inline std::vector<int> AddVector(std::vector<int> a,std::vector<int> b)
  {
    std::vector<int> c;
    for (int i = 0 ; i < a.size(); i++)
    {
      c.push_back(a[i]);
    }
    for (int i = 0 ; i < b.size(); i++)
    {
      c.push_back(b[i]);
    }
    return c;
  }

  class Stats
  {
  public:
    Stats(void)
    {
    }

    virtual ~Stats(void)
    {
    }

    virtual double GetValue(const char id[]);
    virtual void SetValue(const char id[], const double t)
    {
      m_values[id] = t;
    }

    virtual double AddValue(const char id[], const double t);
    virtual void PrintValues(FILE *out) const;

    static Stats* GetSingleton(void)
    {
      return m_singleton;
    }



  protected:
    static Stats *m_singleton;

    UseMap(const std::string, double) m_values;
  };

}

#endif
