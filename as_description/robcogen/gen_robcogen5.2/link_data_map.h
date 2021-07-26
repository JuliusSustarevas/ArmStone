#ifndef RCG_ARMSTONE_LINK_DATA_MAP_H_
#define RCG_ARMSTONE_LINK_DATA_MAP_H_

#include "declarations.h"

namespace armstone {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE_LINK_FOOTPRINT] = rhs[BASE_LINK_FOOTPRINT];
    data[OMNIWHEEL_FL] = rhs[OMNIWHEEL_FL];
    data[OMNIWHEEL_FR] = rhs[OMNIWHEEL_FR];
    data[OMNIWHEEL_BL] = rhs[OMNIWHEEL_BL];
    data[OMNIWHEEL_BR] = rhs[OMNIWHEEL_BR];
    data[XARMLINK1] = rhs[XARMLINK1];
    data[XARMLINK2] = rhs[XARMLINK2];
    data[XARMLINK3] = rhs[XARMLINK3];
    data[XARMLINK4] = rhs[XARMLINK4];
    data[XARMLINK5] = rhs[XARMLINK5];
    data[XARMLINK6] = rhs[XARMLINK6];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE_LINK_FOOTPRINT] = value;
    data[OMNIWHEEL_FL] = value;
    data[OMNIWHEEL_FR] = value;
    data[OMNIWHEEL_BL] = value;
    data[OMNIWHEEL_BR] = value;
    data[XARMLINK1] = value;
    data[XARMLINK2] = value;
    data[XARMLINK3] = value;
    data[XARMLINK4] = value;
    data[XARMLINK5] = value;
    data[XARMLINK6] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base_link_footprint = "
    << map[BASE_LINK_FOOTPRINT]
    << "   omniwheel_fl = "
    << map[OMNIWHEEL_FL]
    << "   omniwheel_fr = "
    << map[OMNIWHEEL_FR]
    << "   omniwheel_bl = "
    << map[OMNIWHEEL_BL]
    << "   omniwheel_br = "
    << map[OMNIWHEEL_BR]
    << "   xarmlink1 = "
    << map[XARMLINK1]
    << "   xarmlink2 = "
    << map[XARMLINK2]
    << "   xarmlink3 = "
    << map[XARMLINK3]
    << "   xarmlink4 = "
    << map[XARMLINK4]
    << "   xarmlink5 = "
    << map[XARMLINK5]
    << "   xarmlink6 = "
    << map[XARMLINK6]
    ;
    return out;
}

}
}
#endif
