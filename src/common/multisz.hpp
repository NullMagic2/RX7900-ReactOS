/*
Module: rx7900.common.multisz
Purpose: Preserve a bounded native filter list while appending one missing name.
Created: 2026-10-22
Architecture: Pure C++17 policy; installer owns registry buffers and writes.
*/
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rx7900 {
enum class MultiStringMerge { unchanged, appended, malformed, too_large };

// Input count includes the terminating empty string. Zero count means a missing
// property. Buffers are separate owned arrays; output becomes usable only when
// the result is unchanged/appended. Failure sets written to zero. Unknown names
// and their order remain unchanged. Only the supplied ASCII service name uses
// case-insensitive matching; Unicode entries are never rewritten or case-folded.
template<typename Character,size_t InputCapacity,size_t FilterLength,size_t OutputCapacity>
MultiStringMerge append_filter_name(const Character (&input)[InputCapacity],size_t input_count,
    const Character (&filter)[FilterLength],Character (&output)[OutputCapacity],size_t& written) noexcept {
    written=0;
    if (static_cast<const void*>(input)==static_cast<const void*>(output) || input_count>InputCapacity ||
        FilterLength<2 || filter[FilterLength-1]!=0) return MultiStringMerge::malformed;
    for (size_t index=0;index+1<FilterLength;++index) {
        if (filter[index]==0 || static_cast<uint32_t>(filter[index])>127) return MultiStringMerge::malformed;
    }
    size_t logical_count=0;
    bool found=false;
    if (input_count!=0) {
        if (input_count<2) return MultiStringMerge::malformed;
        size_t position=0;
        bool terminated=false;
        while (position<input_count) {
            if (input[position]==0) {
                for (size_t tail=position;tail<input_count;++tail) {
                    if (input[tail]!=0) return MultiStringMerge::malformed;
                }
                logical_count=position==0 ? 2 : position+1;
                terminated=true;
                break;
            }
            const size_t first=position;
            while (position<input_count && input[position]!=0) ++position;
            if (position==input_count) return MultiStringMerge::malformed;
            bool matches=position-first==FilterLength-1;
            for (size_t index=0;matches && index+1<FilterLength;++index) {
                const auto fold=[](Character value) noexcept {
                    return value>=static_cast<Character>('A') && value<=static_cast<Character>('Z') ?
                        static_cast<Character>(value+static_cast<Character>('a'-'A')) : value;
                };
                matches=fold(input[first+index])==fold(filter[index]);
            }
            found=found || matches;
            ++position;
        }
        if (!terminated) return MultiStringMerge::malformed;
    }
    const size_t prefix=logical_count<=2 ? 0 : logical_count-1;
    if (found) {
        if (logical_count>OutputCapacity) return MultiStringMerge::too_large;
        for (size_t index=0;index<logical_count;++index) output[index]=input[index];
        written=logical_count;
        return MultiStringMerge::unchanged;
    }
    if (prefix>OutputCapacity || FilterLength>OutputCapacity-prefix ||
        OutputCapacity-prefix-FilterLength<1) return MultiStringMerge::too_large;
    for (size_t index=0;index<prefix;++index) output[index]=input[index];
    for (size_t index=0;index<FilterLength;++index) output[prefix+index]=filter[index];
    output[prefix+FilterLength]=0;
    written=prefix+FilterLength+1;
    return MultiStringMerge::appended;
}
} // namespace rx7900
