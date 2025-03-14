#ifndef TYPE_TRAITS_H_
#define TYPE_TRAITS_H_

namespace traits {
    template <typename...> // can accept many typenames
    using void_t = void;

    /*
    Explanation: 
    - static is per template point. All templates of PointXYZ will have same values as below. And classes with PointXY will have different template instantiation. Without static, each tree will have own copy of variables which is unecessary, because this is a property of Point, not each instance. 
    - constexpr makes the compiler check function at compile time, which is better to catch this check than in runtime. Also, compiler can eliminate branches that know are false at compile time. e.g. the first if statement will be deleted. And can use static_assert. 
    if constexpr (False) {} else {} 
    */

    /*
    - Use two structs so can check the type at compile time. 
    - Can create different structs depending on templates provided
    - the "std::false_type" is a class that we are inheriting that has an attribute value = false. By setting as false_type, the struct automatically has attribute static constexpr false. can be checked at compile time which is what we want. 
    
    */
    // Default case there is no .x. Meaning, if receive HasX with typename PointT and then void, then automatically have false_type
    template <typename PointT, typename = void>
    struct HasX : std::false_type {};

    // this template uses SFINAE (Substitution Failure Is Not An Error)
    /*
    - If the PointT has the X, then the inside evaluates to true, meaning that you now have the previous tempalte HasX<typename PointT, void> and this new that is more specialized HasX <typenamePointT, void>. when have two same templates, the compiler chooses the most specialized one (because is more restrictive because applies to classes that only have .x value), meaning the one below. So, when try to match and see that this below evaluates to true, then because is more specialized then traits has value = true;
    - If evaluation fails it doesnt cause an error, what it does is just deletes the function from the list of candidates, gets deleted. 
    */
    template <typename PointT>
    // std::declval<PointT>().x -> pretendet have instance PointT and access x
    // decltype -> get type of that expression
    // std::void -> if is true inside expression, evaluate void
    struct HasX<PointT, void_t<decltype(std::declval<PointT>().x)>> : std::true_type {};

    /*
    - In summary, when doing HasY<PointT>, this template can be match to both structs below. One that has value = false, and other value = true. How compiler chooses? Well, it evaluates the typename without void, and if the expressions ".y" exists, means that attributes exists. And because this is more specialized, because has the constrain .y value. If tha texpression was invalid, then it will be removed from candidate list and the previous one will be choosen. 
    - If this evaluates to true, then can obtain the value from HasY<PointT>::value because value is an static constexpr attribute, so is can always be accessed
    */
    // Check if has y value
    template <typename PointT, typename = void>
    struct HasY : std::false_type {}; 

    template <typename PointT>
    struct HasY<PointT, void_t<decltype(std::declval<PointT>().y)>> : std::true_type {};

    // Check if has z value
    template <typename PointT, typename = void>
    struct HasZ : std::false_type {}; 

    template <typename PointT>
    struct HasY<PointT, void_t<decltype(std::declval<PointT>().z)>> : std::true_type {};

    template <typename PointT>
    constexpr int getDimensions() {
        if constexpr (HasX<PointT>::value && HasY<PointT>::value && HasZ<PointT>::value)
            return 3;
        else if constexpr (!HasZ<PointT>::value){
            return 2;}
        else {
            return 1;}
    }
}

#endif