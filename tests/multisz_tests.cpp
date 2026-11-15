/*
Module: rx7900.tests.multisz
Purpose: Verify bounded filter repair preserves unknown registry list entries.
Created: 2026-10-22
Architecture: Pure policy fixture; no registry, service or PnP mutation.
*/
#include "common/multisz.hpp"
#include <stdio.h>

namespace {
unsigned checks=0,failures=0;
void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",description);}
}
template<size_t N,size_t M>
bool same(const char16_t (&left)[N],const char16_t (&right)[M],size_t count) {
    if (count>N || count>M) return false;
    for (size_t index=0;index<count;++index) if (left[index]!=right[index]) return false;
    return true;
}
}

int main() {
    using rx7900::append_filter_name;
    using rx7900::MultiStringMerge;
    constexpr char16_t filter[]=u"LightningNT5";
    char16_t output[128]{};
    size_t written=99;
    const char16_t absent[2]{};
    const char16_t expected_single[]=u"LightningNT5\0";
    expect(append_filter_name(absent,0,filter,output,written)==MultiStringMerge::appended &&
        same(output,expected_single,written) && written==sizeof(expected_single)/sizeof(char16_t),
        "missing property creates a complete double-terminated filter list");
    expect(append_filter_name(absent,2,filter,output,written)==MultiStringMerge::appended &&
        same(output,expected_single,written),"empty existing list receives the required filter");
    const char16_t unknown[]=u"OtherFilter\0\u00c9xtra\0";
    const char16_t expected[]=u"OtherFilter\0\u00c9xtra\0LightningNT5\0";
    expect(append_filter_name(unknown,sizeof(unknown)/sizeof(char16_t),filter,output,written)==MultiStringMerge::appended &&
        same(output,expected,written),"unknown ASCII and Unicode filters remain unchanged in their original order");
    const char16_t present[]=u"OtherFilter\0lIgHtNiNgNt5\0Tail\0";
    expect(append_filter_name(present,sizeof(present)/sizeof(char16_t),filter,output,written)==MultiStringMerge::unchanged &&
        same(output,present,written),"existing filter matches case-insensitively without rewriting names");
    const char16_t duplicates[]=u"LightningNT5\0LightningNT5\0";
    expect(append_filter_name(duplicates,sizeof(duplicates)/sizeof(char16_t),filter,output,written)==MultiStringMerge::unchanged &&
        same(output,duplicates,written),"repair adds no further duplicate and preserves the observed list");
    const char16_t prefix[]=u"LightningNT5Extra\0";
    expect(append_filter_name(prefix,sizeof(prefix)/sizeof(char16_t),filter,output,written)==MultiStringMerge::appended,
        "a prefix is not mistaken for the exact filter service");
    char16_t exact[sizeof(expected_single)/sizeof(char16_t)]{};
    expect(append_filter_name(absent,0,filter,exact,written)==MultiStringMerge::appended &&
        same(exact,expected_single,written),"exact output capacity includes both terminators");
    char16_t short_output[sizeof(expected_single)/sizeof(char16_t)-1]{};
    expect(append_filter_name(absent,0,filter,short_output,written)==MultiStringMerge::too_large && written==0,
        "insufficient output capacity never publishes a truncated filter list");
    for (size_t count=1;count<sizeof(unknown)/sizeof(char16_t);++count) {
        expect(append_filter_name(unknown,count,filter,output,written)==MultiStringMerge::malformed && written==0,
            "every nonterminated partial list fails before output is usable");
    }
    const char16_t hidden[]={u'A',0,0,u'B',0,0};
    expect(append_filter_name(hidden,6,filter,output,written)==MultiStringMerge::malformed && written==0,
        "hidden names after an empty terminator are rejected");
    const char16_t invalid_filter[]={u'A',0,u'B',0};
    expect(append_filter_name(absent,0,invalid_filter,output,written)==MultiStringMerge::malformed && written==0,
        "filter identity cannot contain hidden appended service names");
    expect(append_filter_name(absent,3,filter,output,written)==MultiStringMerge::malformed && written==0,
        "reported input length cannot escape its owned array");
    printf("Bounded MULTI_SZ filter repair: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}
