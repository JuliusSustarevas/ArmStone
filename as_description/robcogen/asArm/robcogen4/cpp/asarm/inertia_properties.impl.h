template <typename TRAIT>
iit::asArm::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base_link = iit::rbd::Vector3d(0.175,0.0,0.03803).cast<Scalar>();
    tensor_base_link.fill(
        Scalar(2.7),
        com_base_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.008853708),
                Scalar(0.0915342),
                Scalar(0.084906496),
                Scalar(3.5E-6),
                Scalar(0.017956674),
                Scalar(-1.67E-6)) );

    com_xarmlink1 = iit::rbd::Vector3d(-0.002,0.02692,-0.01332).cast<Scalar>();
    tensor_xarmlink1.fill(
        Scalar(2.16),
        com_xarmlink1,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0073428256),
                Scalar(0.0052897725),
                Scalar(0.0046896925),
                Scalar(-1.272444E-4),
                Scalar(5.5907403E-5),
                Scalar(-0.0015675207)) );

    com_xarmlink2 = iit::rbd::Vector3d(0.03531,-0.21398,0.03386).cast<Scalar>();
    tensor_xarmlink2.fill(
        Scalar(1.71),
        com_xarmlink2,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.105124444),
                Scalar(0.008948017),
                Scalar(0.10430682),
                Scalar(-0.008613624),
                Scalar(0.0027224403),
                Scalar(-0.016962022)) );

    com_xarmlink3 = iit::rbd::Vector3d(0.06781,0.10749,0.01457).cast<Scalar>();
    tensor_xarmlink3.fill(
        Scalar(1.384),
        com_xarmlink3,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.021654077),
                Scalar(0.0099000055),
                Scalar(0.027372088),
                Scalar(0.008669333),
                Scalar(0.0022883206),
                Scalar(0.003859303)) );

    com_xarmlink4 = iit::rbd::Vector3d(-2.1E-4,0.025779976,-0.025379976).cast<Scalar>();
    tensor_xarmlink4.fill(
        Scalar(1.115),
        com_xarmlink4,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0058518867),
                Scalar(0.004725969),
                Scalar(0.0018442962),
                Scalar(-5.631638E-5),
                Scalar(-7.797278E-6),
                Scalar(-0.0011829191)) );

    com_xarmlink5 = iit::rbd::Vector3d(0.05428,0.01781,0.00543).cast<Scalar>();
    tensor_xarmlink5.fill(
        Scalar(1.275),
        com_xarmlink5,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0016447763),
                Scalar(0.0060817497),
                Scalar(0.006847581),
                Scalar(7.4014865E-4),
                Scalar(7.6726405E-4),
                Scalar(2.4680307E-4)) );

    com_xarmlink6 = iit::rbd::Vector3d(0.0,6.048977E-5,0.07606634).cast<Scalar>();
    tensor_xarmlink6.fill(
        Scalar(1.1596),
        com_xarmlink6,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0108265085),
                Scalar(0.010829282),
                Scalar(0.0013922598),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(-6.6778057E-7)) );

}

